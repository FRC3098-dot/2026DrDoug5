package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.IndexConstants;

/**
 * IndexSubsystem
 *
 * Controls game piece movement from intake to shooter.
 *
 * Design rules:
 * - Subsystem NEVER schedules commands
 * - Subsystem NEVER decides WHEN to run
 * - RobotContainer owns all timing and ownership
 *
 * Motors:
 * - Star wheel (final positioning)
 * - Back indexer
 * - Agitator
 */
public class IndexSubsystem extends SubsystemBase {

  /* ==================== Motors ==================== */
  private final SparkMax starMotor =
      new SparkMax(IndexConstants.kIndexerStarMotorID, MotorType.kBrushless);
  private final SparkMax backMotor =
      new SparkMax(IndexConstants.kIndexerBackMotorID, MotorType.kBrushless);
  private final SparkMax agitatorMotor =
      new SparkMax(IndexConstants.kIndexerAgitatorMotorID, MotorType.kBrushless);

  /* ==================== State ==================== */
  private double starPct = 0.0;
  private double backPct = 0.0;
  private double agitatorPct = 0.0;

  /* ==================== Tuning (Elastic / Shuffleboard) ==================== */
  private final ShuffleboardTab indexTab =
      Shuffleboard.getTab("Indexer Tuning");

  private final GenericEntry starClosePctEntry;
  private final GenericEntry starFarPctEntry;
  private final GenericEntry starPassPctEntry;

  /* ==================== Constructor ==================== */
  public IndexSubsystem() {

    // -------- Star motor config --------
    SparkMaxConfig starCfg = new SparkMaxConfig();
    starCfg.inverted(IndexConstants.kIndexerStarMotorInverted);
    starCfg.idleMode(IdleMode.kCoast);
    starCfg.smartCurrentLimit(IndexConstants.kIndexerStarCurrLim);

    // -------- Back motor config --------
    SparkMaxConfig backCfg = new SparkMaxConfig();
    backCfg.inverted(IndexConstants.kIndexerBackMotorInverted);
    backCfg.idleMode(IdleMode.kCoast);
    backCfg.smartCurrentLimit(IndexConstants.kIndexerBackCurrLim);

    // -------- Agitator motor config --------
    SparkMaxConfig agitatorCfg = new SparkMaxConfig();
    agitatorCfg.inverted(IndexConstants.kIndexerAgitatorMotorInverted);
    agitatorCfg.idleMode(IdleMode.kCoast);
    agitatorCfg.smartCurrentLimit(IndexConstants.kIndexerAgitatorCurrLim);

    starMotor.configure(
        starCfg,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    backMotor.configure(
        backCfg,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    agitatorMotor.configure(
        agitatorCfg,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    // ---- Publish tunables immediately (Elastic-safe) ----
    starClosePctEntry = indexTab.add("Star Close Pct", 0.35).getEntry();
    starFarPctEntry   = indexTab.add("Star Far Pct",   0.45).getEntry();
    starPassPctEntry  = indexTab.add("Star Pass Pct",  0.25).getEntry();
  }

  /* ==================== Tuned getters ==================== */
  private static double clampPct(double pct) {
    return MathUtil.clamp(pct, -1.0, 1.0);
  }

  public double getStarClosePct() {
    return clampPct(starClosePctEntry.getDouble(0.35));
  }

  public double getStarFarPct() {
    return clampPct(starFarPctEntry.getDouble(0.45));
  }

  public double getStarPassPct() {
    return clampPct(starPassPctEntry.getDouble(0.25));
  }

  /* ==================== Control API ==================== */

  /**
   * Run all indexer motors with explicit speeds.
   *
   * @param starPct     Star wheel speed [-1, 1]
   * @param backPct     Back indexer speed [-1, 1]
   * @param agitatorPct Agitator speed [-1, 1]
   */
  public void runIndexers(double starPct, double backPct, double agitatorPct) {
    this.starPct = clampPct(starPct);
    this.backPct = clampPct(backPct);
    this.agitatorPct = clampPct(agitatorPct);

    starMotor.set(this.starPct);
    backMotor.set(this.backPct);
    agitatorMotor.set(this.agitatorPct);
  }

  /** Stop all indexer motors. */
  public void stopAll() {
    runIndexers(0.0, 0.0, 0.0);
  }

  /** @return true if any indexer motor is commanded to move */
  public boolean isRunning() {
    return Math.abs(starPct) > 0.05
        || Math.abs(backPct) > 0.05
        || Math.abs(agitatorPct) > 0.05;
  }

  /* ==================== Periodic ==================== */
  @Override
  public void periodic() {
    Logger.recordOutput("Indexer/Command/StarPct", starPct);
    Logger.recordOutput("Indexer/Command/BackPct", backPct);
    Logger.recordOutput("Indexer/Command/AgitatorPct", agitatorPct);

    Logger.recordOutput("Indexer/Tuning/StarClosePct", getStarClosePct());
    Logger.recordOutput("Indexer/Tuning/StarFarPct",   getStarFarPct());
    Logger.recordOutput("Indexer/Tuning/StarPassPct",  getStarPassPct());

    Logger.recordOutput("Indexer/IsRunning", isRunning());
  }
}