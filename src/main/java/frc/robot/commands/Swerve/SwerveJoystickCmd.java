/**
 * Joystick-driven swerve command.
 *
 * <p>Reads joystick inputs (translation + rotation) and sends velocity/angle targets to the
 * swerve subsystem. Applies deadbands and slew-rate limiting. This class should not contain
 * hardware references; it composes the SwerveSubsystem instead.
 */
package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.Constants.VisionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

// Ignore unused variable warnings
@SuppressWarnings("unused")

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private Pose2d startingPose;
     
    
     
     public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        startingPose = swerveSubsystem.getPose(); // Gets X & Y Position
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV(VisionConstants.CameraName)) {
            LimelightHelpers.setLEDMode_ForceBlink(VisionConstants.CameraName);}
        else {
            LimelightHelpers.setLEDMode_ForceOff(VisionConstants.CameraName);}

        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();
        boolean m_fieldOrientation = fieldOrientedFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
            
        // 4. Create chassis speeds
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = new ChassisSpeeds(xSpeed,ySpeed,turningSpeed);
        swerveSubsystem.drive(chassisSpeeds,m_fieldOrientation);
    }
        
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
