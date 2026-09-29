package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.Constants;
import frc.robot.util.Constants.DriveConstants;

/**
 * SwerveSubsystem
 *
 * Responsibilities:
 *  - Own all four swerve modules
 *  - Handle field-relative vs robot-relative driving
 *  - Maintain odometry (Pose2d)
 *  - Integrate PathPlanner AutoBuilder
 *  - Publish drivetrain telemetry to AdvantageScope
 *
 * Lifecycle behavior:
 *  - BRAKE mode when robot is enabled (teleop/auto)
 *  - COAST mode when robot is disabled (pit-safe)
 */
public class SwerveSubsystem extends SubsystemBase {

  /* ==================== Modules ==================== */
  private final SwerveModule frontLeft =
      new SwerveModule(
          DriveConstants.kFLDMotorPort,
          DriveConstants.kFLTMotorPort,
          DriveConstants.kFLDEncReversed,
          DriveConstants.kFLTEncReversed,
          DriveConstants.kFLTAbsEncPort,
          DriveConstants.kFLTAbsEncOffsetRot,
          DriveConstants.kFLTAbsEncReversed,
          "FrontLeft");

  private final SwerveModule frontRight =
      new SwerveModule(
          DriveConstants.kFRDMotorPort,
          DriveConstants.kFRTMotorPort,
          DriveConstants.kFRDEncReversed,
          DriveConstants.kFRTEncReversed,
          DriveConstants.kFRTAbsEncPort,
          DriveConstants.kFRTAbsEncOffsetRot,
          DriveConstants.kFRTAbsEncReversed,
          "FrontRight");

  private final SwerveModule backLeft =
      new SwerveModule(
          DriveConstants.kBLDMotorPort,
          DriveConstants.kBLTMotorPort,
          DriveConstants.kBLDEncReversed,
          DriveConstants.kBLTEncReversed,
          DriveConstants.kBLTAbsEncPort,
          DriveConstants.kBLTAbsEncOffsetRot,
          DriveConstants.kBLTAbsEncReversed,
          "BackLeft");

  private final SwerveModule backRight =
      new SwerveModule(
          DriveConstants.kBRDMotorPort,
          DriveConstants.kBRTMotorPort,
          DriveConstants.kBRDEncReversed,
          DriveConstants.kBRTEncReversed,
          DriveConstants.kBRTAbsEncPort,
          DriveConstants.kBRTAbsEncOffsetRot,
          DriveConstants.kBRTAbsEncReversed,
          "BackRight");

  /* ==================== Sensors ==================== */
  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  /* ==================== Odometry / Field ==================== */
  private final Field2d field = new Field2d();

  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          getRotation2d(),
          getModulePositions(),
          new Pose2d());

  /* ==================== X-Mode ==================== */
  private boolean xModeActive = false;

  /* ==================== Simulation Support ==================== */
  private double simHeadingDeg = 0.0;

  // Store last commanded robot-relative speeds (used for sim integration)
  private ChassisSpeeds lastRobotRelativeSpeeds = new ChassisSpeeds();

  /* ==================== Constructor ==================== */
  public SwerveSubsystem() {

    // Publish Field2d (works for Shuffleboard + Glass)
    SmartDashboard.putData("Field", field);

    ShuffleboardTab fieldTab = Shuffleboard.getTab("Field");
    fieldTab.add("Robot", field).withPosition(0, 0).withSize(6, 4);

    /* ---------- PathPlanner AutoBuilder ---------- */
    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotRelativeSpeeds,
          this::driveRobotRelative,
          new PPHolonomicDriveController(
              Constants.AutoConstants.translationConstants,
              Constants.AutoConstants.rotationConstants),
          config,
          () -> DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
          this);

    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to configure PathPlanner AutoBuilder",
          e.getStackTrace());
    }

    PathPlannerLogging.setLogActivePathCallback(
        poses -> field.getObject("path").setPoses(poses));
  }

  /* ==================== Brake / Coast ==================== */

  /** Call from teleopInit / autonomousInit. */
  public void setBrakeMode() {
    frontLeft.setBrakeMode();
    frontRight.setBrakeMode();
    backLeft.setBrakeMode();
    backRight.setBrakeMode();
  }

  /** Call from disabledInit. */
  public void setCoastMode() {
    frontLeft.setCoastMode();
    frontRight.setCoastMode();
    backLeft.setCoastMode();
    backRight.setCoastMode();
  }

  /* ==================== Gyro ==================== */

  public void zeroHeading() {
    gyro.reset();
    simHeadingDeg = 0.0;
  }

  /** Heading in degrees (-180..180]. */
  public double getHeadingDeg() {
    if (RobotBase.isSimulation()) {
      return simHeadingDeg;
    }
    return Math.IEEEremainder(-gyro.getAngle(), 360.0);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDeg());
  }

  /* ==================== Odometry ==================== */

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  /* ==================== Driving ==================== */

  /**
   * Drive with chassis speeds.
   *
   * @param speeds desired chassis speeds
   * @param fieldRelative true for field-relative control
   */
  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {

    // Prevent drive commands while X-mode is active
    if (xModeActive) {
      lastRobotRelativeSpeeds = new ChassisSpeeds();
      return;
    }

    ChassisSpeeds robotRelative =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                getRotation2d())
            : speeds;

    lastRobotRelativeSpeeds = robotRelative;

    SwerveModuleState[] states =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(robotRelative);

    setModuleStates(states);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states,
        DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]);
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);
  }

  /** X-mode wheel lock (call repeatedly while button is held). */
  public void setX() {
    xModeActive = true;
    lastRobotRelativeSpeeds = new ChassisSpeeds();

    frontLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)));
  }

  public void stopModules() {
    xModeActive = false;
    lastRobotRelativeSpeeds = new ChassisSpeeds();

    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void resetAllEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
  }

  /* ==================== Helpers ==================== */

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(),
      frontRight.getPosition(),
      backLeft.getPosition(),
      backRight.getPosition()
    };
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
    };
  }

  /* ==================== PathPlanner Hooks ==================== */

  /** MUST return robot-relative speeds. */
  private ChassisSpeeds getRobotRelativeSpeeds() {
    if (RobotBase.isSimulation()) {
      return lastRobotRelativeSpeeds;
    }
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /** Called by PathPlanner with robot-relative speeds. */
  private void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  /* ==================== Periodic ==================== */

  @Override
  public void periodic() {
    odometry.update(getRotation2d(), getModulePositions());
    field.setRobotPose(getPose());

    Logger.recordOutput("Swerve/Pose", getPose());
    Logger.recordOutput("Swerve/HeadingDeg", getHeadingDeg());
    Logger.recordOutput("Swerve/ModuleStates", getModuleStates());
    Logger.recordOutput("Swerve/ModulePositions", getModulePositions());
    Logger.recordOutput("Swerve/XModeActive", xModeActive);
    Logger.recordOutput("Swerve/LastCmdRobotRel", lastRobotRelativeSpeeds);

    frontLeft.updateLog();
    frontRight.updateLog();
    backLeft.updateLog();
    backRight.updateLog();
  }

  /* ==================== Simulation ==================== */

  @Override
  public void simulationPeriodic() {
    double dt = 0.02;
    ChassisSpeeds speeds = lastRobotRelativeSpeeds;

    Pose2d newPose =
        getPose().exp(
            new Twist2d(
                speeds.vxMetersPerSecond * dt,
                speeds.vyMetersPerSecond * dt,
                speeds.omegaRadiansPerSecond * dt));

    simHeadingDeg += speeds.omegaRadiansPerSecond * dt * 180.0 / Math.PI;

    // Force odometry pose update in sim
    odometry.resetPosition(getRotation2d(), getModulePositions(), newPose);
  }
}