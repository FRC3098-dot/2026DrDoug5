package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.Constants.IntakeExtendConstants;

/**
 * IntakeExtendSubsystem
 *
 * Features:
 *  - Position control using ProfiledPIDController (deg-based)
 *  - Gravity-neutral behavior: slows/clamps when gravity assists the commanded motion
 *  - Near-stow soft zone clamp to prevent slamming into hard stop
 *  - Oscillation ("waggle") mode between kOscillateMinDeg and kOscillateMaxDeg
 *  - Calibration helper: setCurrentAngleDeg(angleDeg) (used for your guarded "set current to 120")
 *
 * Policy:
 *  - For normal setTargetAngleDeg() moves, we do NOT hold position at target; we coast to rest.
 *  - For oscillation, we keep position mode active (do NOT auto-disengage atGoal).
 */
public class IntakeExtendSubsystem extends SubsystemBase {

  /* ===================== Hardware ===================== */
  private final SparkMax motor =
      new SparkMax(IntakeExtendConstants.kIntakeExtendMotor, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();

  /* ===================== Controller ===================== */
  private final ProfiledPIDController controller;

  private final TrapezoidProfile.Constraints fastConstraints;
  private final TrapezoidProfile.Constraints slowConstraints;

  private boolean positionMode = false;
  private double targetDeg = IntakeExtendConstants.kMinAngleDeg;

  /* ===================== Manual ===================== */
  private double lastManualPct = 0.0;

  /* ===================== Oscillation ===================== */
  private boolean oscillateEnabled = false;
  private boolean oscillatingUp = true;
  private double lastSwitchTimeSec = 0.0;

  // Track last gravity-help state to avoid constantly resetting constraints
  private boolean lastGravityHelped = false;

  public IntakeExtendSubsystem() {

    // Motor configuration
    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.inverted(IntakeExtendConstants.kIntakeExtendInverted);

    // Intake arm policy: COAST (no hard holding)
    cfg.idleMode(IdleMode.kCoast);

    cfg.smartCurrentLimit(IntakeExtendConstants.kIntakeExtendCurrentLimit);

    // Encoder conversion: motor rotations -> degrees
    cfg.encoder.positionConversionFactor(IntakeExtendConstants.kArmDegPerMotorRev);
    cfg.encoder.velocityConversionFactor(IntakeExtendConstants.kArmDegPerMotorRev / 60.0); // deg/sec

    motor.configure(
        cfg,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    // Motion constraints
    fastConstraints = new TrapezoidProfile.Constraints(
        IntakeExtendConstants.kMaxVelDegPerSec,
        IntakeExtendConstants.kMaxAccelDegPerSec2);

    slowConstraints = new TrapezoidProfile.Constraints(
        IntakeExtendConstants.kMaxVelDegPerSec * IntakeExtendConstants.kGravityAssistConstraintScale,
        IntakeExtendConstants.kMaxAccelDegPerSec2 * IntakeExtendConstants.kGravityAssistConstraintScale);

    controller = new ProfiledPIDController(
        IntakeExtendConstants.kArmPosP,
        IntakeExtendConstants.kArmPosI,
        IntakeExtendConstants.kArmPosD,
        fastConstraints);

    controller.setTolerance(IntakeExtendConstants.kArmToleranceDeg);

    // Initialize controller to current position
    controller.reset(getAngleDeg());
    controller.setGoal(getAngleDeg());

    lastGravityHelped = false;
  }

  /* ===================== Public API ===================== */

  public double getAngleDeg() {
    return encoder.getPosition();
  }

  public double getVelocityDegPerSec() {
    return encoder.getVelocity();
  }

  public double getTargetDeg() {
    return targetDeg;
  }

  public boolean atTarget() {
    return controller.atGoal();
  }

  /** Move toward target angle in degrees. */
  public void setTargetAngleDeg(double deg) {
    oscillateEnabled = false;
    positionMode = true;
    lastManualPct = 0.0;

    targetDeg = MathUtil.clamp(deg,
        IntakeExtendConstants.kMinAngleDeg,
        IntakeExtendConstants.kMaxAngleDeg);

    controller.setGoal(targetDeg);
  }

  /** Manual open-loop jog. */
  public void runManual(double pct) {
    oscillateEnabled = false;
    positionMode = false;

    pct = MathUtil.clamp(pct, -1.0, 1.0);
    double a = getAngleDeg();

    // Soft limits in manual
    if ((a <= IntakeExtendConstants.kMinAngleDeg && pct < 0.0)
        || (a >= IntakeExtendConstants.kMaxAngleDeg && pct > 0.0)) {
      pct = 0.0;
    }

    lastManualPct = pct;
    motor.set(pct);
  }

  /** Stop output. */
  public void stop() {
    oscillateEnabled = false;
    positionMode = false;
    lastManualPct = 0.0;
    motor.set(0.0);
  }

  /** Zero encoder at stow (assumes physically at stow). */
  public void zeroAtStow() {
    encoder.setPosition(IntakeExtendConstants.kMinAngleDeg);
    controller.reset(IntakeExtendConstants.kMinAngleDeg);
    targetDeg = IntakeExtendConstants.kMinAngleDeg;
    controller.setGoal(targetDeg);
  }

  /**
   * Calibration helper: declare the CURRENT physical position to be the given angle (deg).
   * Use ONLY when the arm is physically at a known reference.
   */
  public void setCurrentAngleDeg(double angleDeg) {
    // Stop motion during calibration
    stop();

    double clamped = MathUtil.clamp(
        angleDeg,
        IntakeExtendConstants.kMinAngleDeg,
        IntakeExtendConstants.kMaxAngleDeg);

    encoder.setPosition(clamped);
    controller.reset(clamped);
    targetDeg = clamped;
    controller.setGoal(clamped);
  }

  /* ===================== Oscillation API ===================== */

  /** Start waggle between kOscillateMinDeg and kOscillateMaxDeg. */
  public void startOscillate() {
    oscillateEnabled = true;
    positionMode = true;
    lastManualPct = 0.0;

    oscillatingUp = true;
    targetDeg = IntakeExtendConstants.kOscillateMaxDeg;
    controller.setGoal(targetDeg);
    lastSwitchTimeSec = Timer.getFPGATimestamp();
  }

  /** Stop waggle and command extend preset. */
  public void stopOscillateAndExtend() {
    oscillateEnabled = false;
    positionMode = true;
    lastManualPct = 0.0;

    targetDeg = MathUtil.clamp(
        IntakeExtendConstants.kAutonExtendAngleDeg,
        IntakeExtendConstants.kMinAngleDeg,
        IntakeExtendConstants.kMaxAngleDeg);

    controller.setGoal(targetDeg);
  }

  /* ===================== Helpers ===================== */

  /**
   * Returns true if gravity helps motion toward the target.
   *
   * Below neutral: gravity assists retract (decreasing angle)
   * Above neutral: gravity assists extend (increasing angle)
   */
  private boolean gravityHelps(double currentDeg, double targetDeg) {
    double neutral = IntakeExtendConstants.kGravityNeutralDeg;

    return (currentDeg > neutral && targetDeg > currentDeg)   // extending above neutral
        || (currentDeg < neutral && targetDeg < currentDeg);  // retracting below neutral
  }

  private boolean inStowSoftZone(double currentDeg) {
    return currentDeg <= (IntakeExtendConstants.kMinAngleDeg + IntakeExtendConstants.kStowSoftZoneDeg);
  }

  private boolean commandingStow() {
    return targetDeg <= (IntakeExtendConstants.kMinAngleDeg + 1.0);
  }

  private double applySoftLimitsToOutput(double outputPct, double currentDeg) {
    if ((currentDeg <= IntakeExtendConstants.kMinAngleDeg && outputPct < 0.0)
        || (currentDeg >= IntakeExtendConstants.kMaxAngleDeg && outputPct > 0.0)) {
      return 0.0;
    }
    return outputPct;
  }

  /* ===================== Periodic ===================== */

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double currentDeg = getAngleDeg();
    double outputPct = 0.0;

    // ---------------- Oscillation logic ----------------
    if (oscillateEnabled) {
      boolean atTop =
          Math.abs(currentDeg - IntakeExtendConstants.kOscillateMaxDeg)
              <= IntakeExtendConstants.kOscillateToleranceDeg;

      boolean atBottom =
          Math.abs(currentDeg - IntakeExtendConstants.kOscillateMinDeg)
              <= IntakeExtendConstants.kOscillateToleranceDeg;

      if (oscillatingUp && atTop
          && (now - lastSwitchTimeSec) > IntakeExtendConstants.kOscillateDwellSec) {
        oscillatingUp = false;
        targetDeg = IntakeExtendConstants.kOscillateMinDeg;
        controller.setGoal(targetDeg);
        lastSwitchTimeSec = now;
      } else if (!oscillatingUp && atBottom
          && (now - lastSwitchTimeSec) > IntakeExtendConstants.kOscillateDwellSec) {
        oscillatingUp = true;
        targetDeg = IntakeExtendConstants.kOscillateMaxDeg;
        controller.setGoal(targetDeg);
        lastSwitchTimeSec = now;
      }
    }

    // ---------------- Position control ----------------
    if (positionMode) {

      boolean helps = gravityHelps(currentDeg, targetDeg);

      // Update constraints only on transition
      if (helps != lastGravityHelped) {
        controller.setConstraints(helps ? slowConstraints : fastConstraints);
        lastGravityHelped = helps;
      }

      // If not oscillating and we're at goal: disengage and coast (no holding)
      if (!oscillateEnabled && controller.atGoal()) {
        positionMode = false;
        motor.set(0.0);
      } else {
        outputPct = controller.calculate(currentDeg);
        outputPct = MathUtil.clamp(outputPct, -1.0, 1.0);

        // Clamp authority when gravity helps
        if (helps) {
          double lim = IntakeExtendConstants.kGravityAssistOutputLimit;
          outputPct = MathUtil.clamp(outputPct, -lim, lim);
        }

        // Near-stow clamp only when commanding stow
        if (inStowSoftZone(currentDeg) && commandingStow()) {
          double lim = IntakeExtendConstants.kNearStowOutputLimit;
          outputPct = MathUtil.clamp(outputPct, -lim, lim);
        }

        outputPct = applySoftLimitsToOutput(outputPct, currentDeg);
        motor.set(outputPct);
      }
    }

    // ---------------- Logging ----------------
    Logger.recordOutput("IntakeExtend/AngleDeg", currentDeg);
    Logger.recordOutput("IntakeExtend/VelocityDegPerSec", getVelocityDegPerSec());
    Logger.recordOutput("IntakeExtend/TargetDeg", targetDeg);
    Logger.recordOutput("IntakeExtend/AtTarget", controller.atGoal());
    Logger.recordOutput("IntakeExtend/OutputPct", positionMode ? outputPct : lastManualPct);

    Logger.recordOutput("IntakeExtend/OscillateEnabled", oscillateEnabled);
    Logger.recordOutput("IntakeExtend/OscillatingUp", oscillatingUp);

    Logger.recordOutput("IntakeExtend/GravityHelps", gravityHelps(currentDeg, targetDeg));
    Logger.recordOutput("IntakeExtend/NearStowZone", inStowSoftZone(currentDeg));
    Logger.recordOutput("IntakeExtend/CommandingStow", commandingStow());

    Logger.recordOutput("IntakeExtend/CurrentAmps", motor.getOutputCurrent());
  }
}