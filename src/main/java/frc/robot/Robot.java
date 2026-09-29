package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Robot
 *
 * Handles robot lifecycle events (disabled, auton, teleop).
 *
 * Architecture rules:
 * - Robot.java owns lifecycle only
 * - RobotContainer owns subsystems
 * - Subsystems own hardware
 *
 * Intake policy:
 * - Intake encoder is zeroed ONCE at boot
 * - Assumes intake is physically STOWED at power-up
 *
 * Swerve policy:
 * - COAST when disabled
 * - BRAKE when enabled (auton + teleop)
 */
public class Robot extends LoggedRobot {

  private RobotContainer robotContainer;
  private Command autonomousCommand;

  // Brownout edge detection
  private boolean wasBrownedOut = false;

  @Override
  public void robotInit() {

    // ==================== AdvantageKit startup ====================
    Logger.addDataReceiver(new NT4Publisher());
    Logger.addDataReceiver(new WPILOGWriter("/u/logs/"));
    
    Logger.start();
    // =============================================================



    robotContainer = new RobotContainer();

    // ✅ Force swerve into COAST immediately at boot (pit-safe)
    robotContainer.setSwerveCoastMode();

    // Start camera (single USB camera)
    CameraServer.startAutomaticCapture();

    // ==================== Intake ZEROING ====================
    // Assumes intake is physically STOWED at boot
    robotContainer.zeroIntakeAtStow();
  }

  @Override
  public void robotPeriodic() {
    // Run command scheduler
    CommandScheduler.getInstance().run();

    // ==================== POWER / BROWNOUT LOGGING ====================
    double battV = RobotController.getBatteryVoltage();
    double rioV = RobotController.getInputVoltage();
    boolean brownNow = RobotController.isBrownedOut();

    Logger.recordOutput("Power/BatteryVoltage", battV);
    Logger.recordOutput("Power/RioInputVoltage", rioV);
    Logger.recordOutput("Power/IsBrownedOut", brownNow);
    Logger.recordOutput("Power/BrownoutEvent", brownNow && !wasBrownedOut);

    wasBrownedOut = brownNow;
  }

  /* ==================== DISABLED ==================== */
  @Override
  public void disabledInit() {
    // Robot disabled -> make swerve easy to push
    robotContainer.setSwerveCoastMode();
  }

  /* ==================== AUTONOMOUS ==================== */
  @Override
  public void autonomousInit() {
    // Robot enabled -> hold swerve firmly
    robotContainer.setSwerveBrakeMode();

    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousExit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /* ==================== TELEOP ==================== */
  @Override
  public void teleopInit() {
    // Robot enabled -> hold swerve firmly
    robotContainer.setSwerveBrakeMode();

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /* ==================== TEST ==================== */
  @Override
  public void testInit() {
    // Cancel everything for safety
    CommandScheduler.getInstance().cancelAll();
  }
}