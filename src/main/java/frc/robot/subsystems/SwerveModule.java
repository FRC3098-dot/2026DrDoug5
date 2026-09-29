
package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.util.Constants.DriveConstants;
import frc.robot.util.Constants.ModuleConstants;

/**
 * SwerveModule
 *
 * - Drive motor: open-loop percent output (speed/maxSpeed)
 * - Turn motor: PID on steering angle (radians), continuous wrap
 * - Absolute encoder: DutyCycleEncoder used to initialize steering angle at boot
 *
 * Brake/Coast:
 * - Default at boot: COAST (pit-safe until Robot enables BRAKE)
 * - setBrakeMode(): drive + turn idle mode = BRAKE
 * - setCoastMode(): drive + turn idle mode = COAST
 *
 * IMPORTANT:
 * - We reuse SparkMaxConfig objects so conversion factors and inversion remain intact.
 * - Call brake/coast ONLY on robot mode transitions (teleopInit/autonInit/disabledInit),
 *   NOT in periodic().
 */
public class SwerveModule {

  // Motors
  private final SparkMax driveMotor;
  private final SparkMax turnMotor;

  // Encoders
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;

  // Absolute encoder
  private final DutyCycleEncoder absEncoder;

  // Turning PID (radians)
  private final PIDController turnPID;

  // Configs (stored so we can change idleMode without losing other settings)
  private final SparkMaxConfig driveCfg = new SparkMaxConfig();
  private final SparkMaxConfig turnCfg = new SparkMaxConfig();

  // Absolute encoder settings
  private final boolean absReversed;
  private final double absOffsetRot; // offset as fraction of rotation (0..1)

  // Name for logging
  private final String name;

  // Cache last applied idle mode to avoid unnecessary reconfig
  private IdleMode currentIdleMode = null;

  public SwerveModule(
      int driveMotorId,
      int turnMotorId,
      boolean driveReversed,
      boolean turnReversed,
      int absEncoderDioPort,
      double absOffsetRot,
      boolean absReversed,
      String name) {

    this.absOffsetRot = absOffsetRot;
    this.absReversed = absReversed;
    this.name = name;

    driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
    turnMotor  = new SparkMax(turnMotorId,  MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder  = turnMotor.getEncoder();

    absEncoder = new DutyCycleEncoder(absEncoderDioPort);

    // Turning PID
    turnPID = new PIDController(ModuleConstants.kPTurning, 0.0, 0.0);
    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    // ---------------- Configure Drive ----------------
    driveCfg.inverted(driveReversed);
    driveCfg.smartCurrentLimit(DriveConstants.kDriveMotorCurrLim);

    // DEFAULT AT BOOT: COAST (Robot.java will switch to BRAKE when enabled)
    driveCfg.idleMode(IdleMode.kCoast);

    driveCfg.encoder.positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveCfg.encoder.velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);

    // ---------------- Configure Turn ----------------
    turnCfg.inverted(turnReversed);
    turnCfg.smartCurrentLimit(DriveConstants.kTurnMotorCurrLim);

    // DEFAULT AT BOOT: COAST (Robot.java will switch to BRAKE when enabled)
    turnCfg.idleMode(IdleMode.kCoast);

    turnCfg.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
    turnCfg.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

    // Apply configs (persist OK during init)
    driveMotor.configure(
        driveCfg,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    turnMotor.configure(
        turnCfg,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    // Track current mode (matches default)
    currentIdleMode = IdleMode.kCoast;

    // Initialize steering encoder from absolute
    resetEncoders();
  }

  /* ========================================================= */
  /* ================= Brake / Coast Switching ================ */
  /* ========================================================= */

  /** Set both drive and turn motors to BRAKE mode. Call in teleopInit/autonInit. */
  public void setBrakeMode() {
    if (currentIdleMode == IdleMode.kBrake) return;
    currentIdleMode = IdleMode.kBrake;

    driveCfg.idleMode(IdleMode.kBrake);
    turnCfg.idleMode(IdleMode.kBrake);

    // Do NOT persist while enabled (REV warning). Runtime behavior only.
    driveMotor.configure(
        driveCfg,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);

    turnMotor.configure(
        turnCfg,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  /** Set both drive and turn motors to COAST mode. Call in disabledInit. */
  public void setCoastMode() {
    if (currentIdleMode == IdleMode.kCoast) return;
    currentIdleMode = IdleMode.kCoast;

    driveCfg.idleMode(IdleMode.kCoast);
    turnCfg.idleMode(IdleMode.kCoast);

    driveMotor.configure(
        driveCfg,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);

    turnMotor.configure(
        turnCfg,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  /* ========================================================= */
  /* ======================= Logging ========================== */
  /* ========================================================= */

  public void updateLog() {
    Logger.recordOutput("Swerve/Module/" + name + "/AbsAngleRad", getAbsoluteAngleRad());
    Logger.recordOutput("Swerve/Module/" + name + "/TurnPosRad", getTurningPositionRad());
    Logger.recordOutput("Swerve/Module/" + name + "/TurnVelRadPerSec", getTurningVelocityRadPerSec());
    Logger.recordOutput("Swerve/Module/" + name + "/DrivePosM", getDrivePositionMeters());
    Logger.recordOutput("Swerve/Module/" + name + "/DriveVelMps", getDriveVelocityMps());

    Logger.recordOutput("Swerve/Module/" + name + "/DriveAmps", driveMotor.getOutputCurrent());
    Logger.recordOutput("Swerve/Module/" + name + "/TurnAmps", turnMotor.getOutputCurrent());
  }

  /* ========================================================= */
  /* ======================= Control ========================== */
  /* ========================================================= */

  /** Explicit non-deprecated optimize (WPILib friendly). */
  private static SwerveModuleState optimizeState(SwerveModuleState desired, Rotation2d currentAngle) {
    Rotation2d delta = desired.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(
          -desired.speedMetersPerSecond,
          desired.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    }
    return desired;
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState = optimizeState(desiredState, getState().angle);

    // Drive open-loop percent output
    double drivePct =
        desiredState.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
    drivePct = MathUtil.clamp(drivePct, -1.0, 1.0);

    // If nearly stopped, command 0 drive but still steer to angle
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      drivePct = 0.0;
    }
    driveMotor.set(drivePct);

    // Turning PID on radians
    double turnOut = turnPID.calculate(getTurningPositionRad(), desiredState.angle.getRadians());
    turnOut = MathUtil.clamp(turnOut, -1.0, 1.0);
    turnMotor.set(turnOut);

    Logger.recordOutput("Swerve/Module/" + name + "/CmdSpeedMps", desiredState.speedMetersPerSecond);
    Logger.recordOutput("Swerve/Module/" + name + "/CmdAngleRad", desiredState.angle.getRadians());
    Logger.recordOutput("Swerve/Module/" + name + "/CmdDrivePct", drivePct);
    Logger.recordOutput("Swerve/Module/" + name + "/CmdTurnPct", turnOut);
  }

  public void stop() {
    driveMotor.set(0.0);
    turnMotor.set(0.0);
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0.0);
    turnEncoder.setPosition(getAbsoluteAngleRad());
  }

  /* ========================================================= */
  /* ====================== Measurements ====================== */
  /* ========================================================= */

  /** Absolute angle in radians wrapped to [-pi, +pi]. */
  public double getAbsoluteAngleRad() {
    double rot = absEncoder.get(); // typically 0..1
    if (absReversed) rot = 1.0 - rot;

    rot -= absOffsetRot; // offset as rotation fraction
    rot = MathUtil.inputModulus(rot, -0.5, 0.5);
    return rot * 2.0 * Math.PI;
  }

  public double getDrivePositionMeters() {
    return driveEncoder.getPosition();
  }

  public double getDriveVelocityMps() {
    return driveEncoder.getVelocity();
  }

  public double getTurningPositionRad() {
    return turnEncoder.getPosition();
  }

  public double getTurningVelocityRadPerSec() {
    return turnEncoder.getVelocity();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocityMps(), new Rotation2d(getTurningPositionRad()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePositionMeters(), new Rotation2d(getTurningPositionRad()));
  }
}