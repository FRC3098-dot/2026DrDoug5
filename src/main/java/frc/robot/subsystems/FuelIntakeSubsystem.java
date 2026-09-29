package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.util.Constants.FuelConstants;

/**
 * FuelIntakeSubsystem
 *
 * Controls the intake roller that pulls game pieces into the robot.
 *
 * Characteristics:
 *  - Single motor
 *  - Open-loop percent output
 *  - High current sensitivity (jams are common)
 *
 * AdvantageKit logging:
 *  - Commanded output
 *  - Motor current (amps)
 *  - Running state
 */
public class FuelIntakeSubsystem extends SubsystemBase {

  /* ==================== Motor ==================== */

  private final SparkMax intakeMotor =
      new SparkMax(FuelConstants.kFuelIntakeMotorID, MotorType.kBrushless);

  /* ==================== State ==================== */

  private double commandedPercent = 0.0;

  /* ==================== Constructor ==================== */

  public FuelIntakeSubsystem() {

    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.inverted(FuelConstants.kFuelIntakeInverted);
    cfg.idleMode(IdleMode.kCoast); // Coast is typical for rollers
    cfg.smartCurrentLimit(FuelConstants.kFuelIntakeCurrLim);

    intakeMotor.configure(
        cfg,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  /* ==================== Control API ==================== */

  /**
   * Run the intake roller.
   *
   * @param percent Output [-1, 1]
   *                Positive direction should pull game pieces IN.
   */
  public void runIntake(double percent) {
    commandedPercent = percent;
    intakeMotor.set(percent);
  }

  /** Stop the intake roller. */
  public void stopIntake() {
    commandedPercent = 0.0;
    intakeMotor.set(0.0);
  }

  /** @return true if intake is commanded to run */
  public boolean isRunning() {
    return Math.abs(commandedPercent) > 0.05;
  }

  /* ==================== Measurements ==================== */

  /** @return motor output current in amps */
  public double getCurrentAmps() {
    return intakeMotor.getOutputCurrent();
  }

  /* ==================== Periodic ==================== */

  @Override
  public void periodic() {

    // ---- AdvantageKit logging ----
    Logger.recordOutput("FuelIntake/Command/Percent", commandedPercent);
    Logger.recordOutput("FuelIntake/Current/Amps", getCurrentAmps());
    Logger.recordOutput("FuelIntake/IsRunning", isRunning());
  }
}