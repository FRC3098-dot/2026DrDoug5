package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.ShooterConstants;

/**
 * ShooterSubsystem
 *
 * RPM-controlled dual-wheel shooter.
 *
 * Design rules:
 * - Subsystem NEVER schedules or owns commands
 * - Subsystem NEVER stops itself except via explicit stopAll()
 * - RobotContainer owns all command lifecycle
 */
public class ShooterSubsystem extends SubsystemBase {

  /* ==================== Motors ==================== */
  private final SparkMax frontMotor =
      new SparkMax(ShooterConstants.kFrontShooterMotorID, MotorType.kBrushless);
  private final SparkMax backMotor =
      new SparkMax(ShooterConstants.kBackShooterMotorID, MotorType.kBrushless);

  /* ==================== Encoders ==================== */
  private final RelativeEncoder frontEncoder = frontMotor.getEncoder();
  private final RelativeEncoder backEncoder  = backMotor.getEncoder();

  /* ==================== Limits ==================== */
  private static final double kMaxFrontRpm = 6000.0;
  private static final double kMaxBackRpm  = 6000.0;

  /* ==================== Targets ==================== */
  private double targetFrontRpm = 0.0;
  private double targetBackRpm  = 0.0;

  /* ==================== Velocity PID ==================== */
  private static final double kFrontVelP = 0.00020;
  private static final double kBackVelP  = 0.00020;

  private final PIDController frontVelPid =
      new PIDController(kFrontVelP, 0.0, 0.0);
  private final PIDController backVelPid =
      new PIDController(kBackVelP, 0.0, 0.0);

  /* ==================== Feed Boost ==================== */
  private boolean feedBoostEnabled = false;
  private static final double kFeedBoostFrontFrac = 0.05;
  private static final double kFeedBoostBackFrac  = 0.05;

  /* ==================== Output ==================== */
  private double commandedFrontPct = 0.0;
  private double commandedBackPct  = 0.0;

  /* ==================== Gating ==================== */
  private static final double kMinTargetToCareRpm = 200.0;

  /* ==================== Tuning (Elastic / Shuffleboard) ==================== */
  private final ShuffleboardTab shooterTab =
      Shuffleboard.getTab("Shooter Tuning");

  private final GenericEntry closeFrontRpmEntry;
  private final GenericEntry closeBackRpmEntry;
  private final GenericEntry farFrontRpmEntry;
  private final GenericEntry farBackRpmEntry;
  private final GenericEntry passFrontRpmEntry;
  private final GenericEntry passBackRpmEntry;

  /* ==================== Constructor ==================== */
  public ShooterSubsystem() {

    SparkMaxConfig frontCfg = new SparkMaxConfig();
    frontCfg.inverted(ShooterConstants.kFrontShooterMotorInverted);
    frontCfg.idleMode(IdleMode.kCoast);
    frontCfg.smartCurrentLimit(ShooterConstants.kFrontShooterCurrLim);

    SparkMaxConfig backCfg = new SparkMaxConfig();
    backCfg.inverted(ShooterConstants.kBackShooterMotorInverted);
    backCfg.idleMode(IdleMode.kCoast);
    backCfg.smartCurrentLimit(ShooterConstants.kBackShooterCurrLim);

    frontMotor.configure(
        frontCfg,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    backMotor.configure(
        backCfg,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    /* ---- Publish tunable RPMs immediately ---- */
    closeFrontRpmEntry = shooterTab.add("Close Front RPM", 2800).getEntry();
    closeBackRpmEntry  = shooterTab.add("Close Back RPM", 220).getEntry();

    farFrontRpmEntry   = shooterTab.add("Far Front RPM", 4000).getEntry();
    farBackRpmEntry    = shooterTab.add("Far Back RPM", 450).getEntry();

    passFrontRpmEntry  = shooterTab.add("Pass Front RPM", 3000).getEntry();
    passBackRpmEntry   = shooterTab.add("Pass Back RPM", 250).getEntry();
  }

  /* ==================== Helpers ==================== */
  private static double clampRpm(double rpm, double maxAbs) {
    return MathUtil.clamp(rpm, -maxAbs, maxAbs);
  }

  public double getCloseFrontRpm() { return clampRpm(closeFrontRpmEntry.getDouble(2800), kMaxFrontRpm); }
  public double getCloseBackRpm()  { return clampRpm(closeBackRpmEntry.getDouble(215),  kMaxBackRpm); }
  public double getFarFrontRpm()   { return clampRpm(farFrontRpmEntry.getDouble(3600),  kMaxFrontRpm); }
  public double getFarBackRpm()    { return clampRpm(farBackRpmEntry.getDouble(300),   kMaxBackRpm); }
  public double getPassFrontRpm()  { return clampRpm(passFrontRpmEntry.getDouble(4000), kMaxFrontRpm); }
  public double getPassBackRpm()   { return clampRpm(passBackRpmEntry.getDouble(1000),  kMaxBackRpm); }

  /* ==================== External API ==================== */

  /** Called by RobotContainer shooter buttons */
  public void runFlywheelsRPM(double frontRpm, double backRpm) {
    System.out.println("SHOOTER CMD: front=" + frontRpm + " back=" + backRpm); 
    targetFrontRpm = clampRpm(frontRpm, kMaxFrontRpm);
    targetBackRpm  = clampRpm(backRpm,  kMaxBackRpm);
  }

  /** Explicit stop ONLY (never automatic) */
  public void stopAll() {
    targetFrontRpm = 0.0;
    targetBackRpm  = 0.0;
    feedBoostEnabled = false;
    frontVelPid.reset();
    backVelPid.reset();
    commandedFrontPct = 0.0;
    commandedBackPct  = 0.0;
    frontMotor.set(0.0);
    backMotor.set(0.0);
  }

  /** Feed latch controls this — shooter does NOT decide */
  public void setFeedBoostEnabled(boolean enabled) {
    feedBoostEnabled = enabled;
  }

  /* ==================== Status ==================== */

  public boolean isShooterActive() {
    return Math.abs(targetFrontRpm) >= kMinTargetToCareRpm
        || Math.abs(targetBackRpm)  >= kMinTargetToCareRpm;
  }

  private double boostedFrontTargetRpm() {
    return feedBoostEnabled
        ? clampRpm(targetFrontRpm * (1.0 + kFeedBoostFrontFrac), kMaxFrontRpm)
        : targetFrontRpm;
  }

  private double boostedBackTargetRpm() {
    return feedBoostEnabled
        ? clampRpm(targetBackRpm * (1.0 + kFeedBoostBackFrac), kMaxBackRpm)
        : targetBackRpm;
  }

  public boolean isAtSpeedFraction(double frac) {
    double fT = Math.abs(boostedFrontTargetRpm());
    double bT = Math.abs(boostedBackTargetRpm());

    if (fT < kMinTargetToCareRpm || bT < kMinTargetToCareRpm) {
      return false;
    }

    return Math.abs(frontEncoder.getVelocity()) >= frac * fT
        && Math.abs(backEncoder.getVelocity())  >= frac * bT;
  }

  /* ==================== Control Loop ==================== */
  @Override
  public void periodic() {
    
   double fT = boostedFrontTargetRpm();
    double bT = boostedBackTargetRpm();
    double fM = frontEncoder.getVelocity();
    double bM = backEncoder.getVelocity();

    if (Math.abs(fT) < kMinTargetToCareRpm
     || Math.abs(bT) < kMinTargetToCareRpm) {

      frontVelPid.reset();
      backVelPid.reset();
      commandedFrontPct = 0.0;
      commandedBackPct  = 0.0;
      frontMotor.set(0.0);
      backMotor.set(0.0);

    } else {

      double fFF   = fT / kMaxFrontRpm;
      double bFF   = bT / kMaxBackRpm;
      double fCorr = frontVelPid.calculate(fM, fT);
      double bCorr = backVelPid.calculate(bM, bT);

      commandedFrontPct = MathUtil.clamp(fFF + fCorr, -1.0, 1.0);
      commandedBackPct  = MathUtil.clamp(bFF + bCorr, -1.0, 1.0);

      frontMotor.set(commandedFrontPct);
      backMotor.set(commandedBackPct);
    }

    /* ---- Logging ---- */
    Logger.recordOutput("Shooter/Active", isShooterActive());
    Logger.recordOutput("Shooter/Target/FrontRPM", fT);
    Logger.recordOutput("Shooter/Target/BackRPM",  bT);
    Logger.recordOutput("Shooter/Measured/FrontRPM", fM);
    Logger.recordOutput("Shooter/Measured/BackRPM",  bM);
    Logger.recordOutput("Shooter/Command/FrontPct", commandedFrontPct);
    Logger.recordOutput("Shooter/Command/BackPct",  commandedBackPct);
    Logger.recordOutput("Shooter/At90Pct", isAtSpeedFraction(0.90));
  }
}