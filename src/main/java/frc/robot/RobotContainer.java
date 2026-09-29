package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.Swerve.SwerveJoystickCmd;
import frc.robot.subsystems.FuelIntakeSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeExtendSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.util.Constants.IndexConstants;
import frc.robot.util.Constants.IntakeExtendConstants;
import frc.robot.util.Constants.OIConstants;
import frc.robot.util.Constants.ShooterConstants;

public class RobotContainer {

  /* ===================== SUBSYSTEMS ===================== */
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IndexSubsystem indexer = new IndexSubsystem();
  private final FuelIntakeSubsystem fuelIntake = new FuelIntakeSubsystem();
  private final IntakeExtendSubsystem intakeExtend = new IntakeExtendSubsystem();

  /* ===================== CONTROLLERS ===================== */
  public final XboxController driverJoystick =
      new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController codriverJoystick =
      new XboxController(OIConstants.kCoDriverControllerPort);

  /* ===================== AUTON ===================== */
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private static final String kDefaultAuto = "LeftShoot Only";
  private static final String kAuto1 = "CentShoot Only";
  private static final String kAuto2 = "RightShoot Only";
  private static final String kAuto3 = "LeftShoot and Cross";
  private static final String kAuto4 = "RightShoot and Cross";

  /* ===================== SHOT MODE (for tuned STAR feed) ===================== */
  private enum ShotMode { NONE, CLOSE, FAR, PASS }
  private ShotMode currentShotMode = ShotMode.NONE;

  private void setShotMode(ShotMode mode) {
    currentShotMode = mode;
  }

  /** Tuned STAR speed by last selected shot mode (defaults to CLOSE). */
  private double getTunedStarPctForMode() {
    switch (currentShotMode) {
      case FAR:  return indexer.getStarFarPct();
      case PASS: return indexer.getStarPassPct();
      case CLOSE:
      default:   return indexer.getStarClosePct();
    }
  }

  /* ===================== DRIVER POV BUMP SPEED ===================== */
  private double getDriverBumpSpeedMps() {
    double rt = driverJoystick.getRawAxis(3);
    return OIConstants.kBumpMinSpeedMps
        + rt * (OIConstants.kBumpMaxSpeedMps - OIConstants.kBumpMinSpeedMps);
  }

  /* ===================== LIMELIGHT ALIGN ===================== */
  private static final String kLimelightTableName = "limelight";
  private final NetworkTable limelight =
      NetworkTableInstance.getDefault().getTable(kLimelightTableName);

  private static final int[] kAllowedAlignTagIds = {10, 25};

  private boolean isAllowedAlignTag(int tagId) {
    for (int id : kAllowedAlignTagIds) {
      if (id == tagId) return true;
    }
    return false;
  }

  private final PIDController alignPid = new PIDController(0.025, 0.0, 0.002);
  private static final double kAlignToleranceDeg = 1.0;
  private static final double kMaxOmegaRadPerSec = 2.5;

  private Command createAlignToTargetCommand() {
    alignPid.setTolerance(kAlignToleranceDeg);
    return new RunCommand(() -> {
      double tv = limelight.getEntry("tv").getDouble(0.0);
      int tid = (int) limelight.getEntry("tid").getDouble(-1.0);

      boolean allowed = tv >= 1.0 && isAllowedAlignTag(tid);
      if (!allowed) {
        swerveSubsystem.drive(new ChassisSpeeds(), false);
        return;
      }

      double tx = limelight.getEntry("tx").getDouble(0.0);
      double omega = MathUtil.clamp(
          alignPid.calculate(tx, 0.0),
          -kMaxOmegaRadPerSec,
          kMaxOmegaRadPerSec);

      swerveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, omega), false);
    }, swerveSubsystem).finallyDo(
        () -> swerveSubsystem.drive(new ChassisSpeeds(), false));
  }

  /* ===================== INDEXER GATING ===================== */
  private static final double kIndexerStartShooterFrac = 0.90;
  private static final double kIndexerOverrideLtThreshold = 0.75;
  private boolean indexerFeedLatched = false;

  private boolean codriverOverrideFeed() {
    return codriverJoystick.getRawAxis(OIConstants.kCodriverLTrigger)
        > kIndexerOverrideLtThreshold;
  }

  // ✅ FIX: proper OR so latch can ever become true
  private boolean canStartIndexerFeed() {
    return codriverOverrideFeed()
        || shooter.isAtSpeedFraction(kIndexerStartShooterFrac);
  }

  /* ===================== CONSTRUCTOR ===================== */
  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindings();
    registerNamedCommands();
    configureAutonChooser();
    buildShuffleboard();
  }

  /* ===================== DEFAULT DRIVE ===================== */
  private void configureDefaultCommands() {
    swerveSubsystem.setDefaultCommand(
        new SwerveJoystickCmd(
            swerveSubsystem,
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
            () -> true));
  }

  /* ===================== BUTTON BINDINGS ===================== */
  private void configureButtonBindings() {

    /* ---------- DRIVER ---------- */
    new JoystickButton(driverJoystick, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(swerveSubsystem::zeroHeading));

    new JoystickButton(driverJoystick, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(swerveSubsystem::setX, swerveSubsystem))
        .onFalse(new InstantCommand(swerveSubsystem::stopModules));

    new JoystickButton(driverJoystick, XboxController.Button.kA.value)
        .whileTrue(createAlignToTargetCommand());

    new Trigger(() -> driverJoystick.getPOV() == 0)
        .whileTrue(new RunCommand(
            () -> swerveSubsystem.drive(
                new ChassisSpeeds(getDriverBumpSpeedMps(), 0.0, 0.0), false),
            swerveSubsystem));

    new Trigger(() -> driverJoystick.getPOV() == 180)
        .whileTrue(new RunCommand(
            () -> swerveSubsystem.drive(
                new ChassisSpeeds(-getDriverBumpSpeedMps(), 0.0, 0.0), false),
            swerveSubsystem));

    new Trigger(() -> driverJoystick.getPOV() == 270)
        .whileTrue(new RunCommand(
            () -> swerveSubsystem.drive(
                new ChassisSpeeds(0.0, getDriverBumpSpeedMps(), 0.0), false),
            swerveSubsystem));

    new Trigger(() -> driverJoystick.getPOV() == 90)
        .whileTrue(new RunCommand(
            () -> swerveSubsystem.drive(
                new ChassisSpeeds(0.0, -getDriverBumpSpeedMps(), 0.0), false),
            swerveSubsystem));

    /* ---------- CODRIVER ---------- */

    // Intake calibration (disabled only)
    new JoystickButton(codriverJoystick, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> {
          if (!DriverStation.isEnabled()) {
            intakeExtend.setCurrentAngleDeg(120.0);
          }
        }, intakeExtend));

    // Feed latch (Indexer only; shooter must remain owned by shooter buttons)
    new JoystickButton(codriverJoystick, OIConstants.kCodriverLBumper)
        .whileTrue(new RunCommand(() -> {

          if (!indexerFeedLatched) {
            indexerFeedLatched = canStartIndexerFeed();
            shooter.setFeedBoostEnabled(indexerFeedLatched);
          }

          if (indexerFeedLatched) {
            // ✅ Use tuned STAR speed; keep back + agitator as your existing constants
            double tunedStar = getTunedStarPctForMode();

            indexer.runIndexers(
                tunedStar,
                IndexConstants.kIndexerBackSpeed,
                IndexConstants.kIndexerAgitatorSpeed  // NOTE: this is now your AgitatorPct input
            );
          } else {
            indexer.stopAll();
          }

        }, indexer))
        .onFalse(new InstantCommand(() -> {
          indexerFeedLatched = false;
          shooter.setFeedBoostEnabled(false);
          indexer.stopAll();
        }));

    // Reverse indexer
    new JoystickButton(codriverJoystick, OIConstants.kCodriver_X)
        .whileTrue(new RunCommand(() -> {
          indexer.runIndexers(
              -IndexConstants.kIndexerStarSpeed,
              -IndexConstants.kIndexerBackSpeed,
              -IndexConstants.kIndexerAgitatorSpeed
          );
        }, indexer))
        .onFalse(new InstantCommand(indexer::stopAll));

    // Reverse shooter + indexer (jam clear): owns BOTH subsystems intentionally
    new JoystickButton(codriverJoystick, OIConstants.kCodriver_B)
        .whileTrue(new RunCommand(() -> {
          shooter.runFlywheelsRPM(
              -shooter.getCloseFrontRpm(),
              -shooter.getCloseBackRpm());
          indexer.runIndexers(
              -IndexConstants.kIndexerStarSpeed,
              -IndexConstants.kIndexerBackSpeed,
              -IndexConstants.kIndexerAgitatorSpeed
          );
        }, shooter, indexer))
        .onFalse(new InstantCommand(() -> {
          shooter.stopAll();
          indexer.stopAll();
          setShotMode(ShotMode.NONE);
        }, shooter, indexer));

    /* ---------- SHOOTER MODES (RPM) ---------- */
    // ✅ FIX: stable RunCommand owner while held + stop once on release

    // FAR (Right Trigger axis)
    new Trigger(() ->
        codriverJoystick.getRawAxis(OIConstants.kCodriverRTrigger)
            > ShooterConstants.kTriggerThreshold)
        .whileTrue(new RunCommand(() -> {
          setShotMode(ShotMode.FAR);
          shooter.runFlywheelsRPM(
              shooter.getFarFrontRpm(),
              shooter.getFarBackRpm());
        }, shooter))
        .onFalse(new InstantCommand(() -> {
          shooter.stopAll();
          setShotMode(ShotMode.NONE);
        }, shooter));

    // CLOSE (RBumper)
    new JoystickButton(codriverJoystick, OIConstants.kCodriverRBumper)
        .whileTrue(new RunCommand(() -> {
          setShotMode(ShotMode.CLOSE);
          shooter.runFlywheelsRPM(
              shooter.getCloseFrontRpm(),
              shooter.getCloseBackRpm());
        }, shooter))
        .onFalse(new InstantCommand(() -> {
          shooter.stopAll();
          setShotMode(ShotMode.NONE);
        }, shooter));

    // PASS (Y)
    new JoystickButton(codriverJoystick, OIConstants.kCodriver_Y)
        .whileTrue(new RunCommand(() -> {
          setShotMode(ShotMode.PASS);
          shooter.runFlywheelsRPM(
              shooter.getPassFrontRpm(),
              shooter.getPassBackRpm());
        }, shooter))
        .onFalse(new InstantCommand(() -> {
          shooter.stopAll();
          setShotMode(ShotMode.NONE);
        }, shooter));

    /* ---------- INTAKE ARM ---------- */
    new Trigger(() -> codriverJoystick.getPOV() == 0)
        .onTrue(new InstantCommand(
            () -> intakeExtend.setTargetAngleDeg(
                IntakeExtendConstants.kAutonExtendAngleDeg),
            intakeExtend));

    new Trigger(() -> codriverJoystick.getPOV() == 180)
        .onTrue(new InstantCommand(
            () -> intakeExtend.setTargetAngleDeg(
                IntakeExtendConstants.kMinAngleDeg),
            intakeExtend));

    Trigger oscLeft = new Trigger(() -> codriverJoystick.getPOV() == 270);
    oscLeft.onTrue(new InstantCommand(intakeExtend::startOscillate, intakeExtend));
    oscLeft.onFalse(new InstantCommand(intakeExtend::stopOscillateAndExtend, intakeExtend));

    new Trigger(() ->
        Math.abs(codriverJoystick.getRawAxis(1)) > 0.25
            && codriverJoystick.getPOV() == -1)
        .whileTrue(new RunCommand(
            () -> intakeExtend.runManual(-codriverJoystick.getRawAxis(1)),
            intakeExtend))
        .onFalse(new InstantCommand(intakeExtend::stop));

    /* ---------- INTAKE ROLLER (BIDIRECTIONAL, FIXED SPEED) ---------- */
    final double kIntakeDeadband = 0.20;
    final double kIntakeFixedSpeed = 0.60;//60% works 


    new Trigger(() -> Math.abs(codriverJoystick.getRawAxis(5)) > kIntakeDeadband)
        .whileTrue(new RunCommand(() -> {
          fuelIntake.runIntake(
              Math.signum(codriverJoystick.getRawAxis(5)) * -kIntakeFixedSpeed);
        }, fuelIntake))
        .onFalse(new InstantCommand(fuelIntake::stopIntake));
  }

  /* ===================== PATHPLANNER ===================== */
  private void registerNamedCommands() {

    NamedCommands.registerCommand(
        "Extend Intake",
        Commands.runOnce(
            () -> intakeExtend.setTargetAngleDeg(
                IntakeExtendConstants.kAutonExtendAngleDeg),
            intakeExtend));

    NamedCommands.registerCommand(
        "Stow Intake",
        Commands.runOnce(
            () -> intakeExtend.setTargetAngleDeg(
                IntakeExtendConstants.kMinAngleDeg),
            intakeExtend));

    NamedCommands.registerCommand(
        "Shoot Close",
        Commands.runOnce(
            () -> shooter.runFlywheelsRPM(
                shooter.getCloseFrontRpm(),
                shooter.getCloseFrontRpm()),
            shooter));

    NamedCommands.registerCommand(
        "Shoot Far",
        Commands.runOnce(
            () -> shooter.runFlywheelsRPM(
                shooter.getFarFrontRpm(),
                shooter.getFarBackRpm()),
            shooter));

    NamedCommands.registerCommand(
        "Shoot Off",
        Commands.runOnce(() -> {
          shooter.stopAll();
          setShotMode(ShotMode.NONE);
        }, shooter));

    NamedCommands.registerCommand(
        "Align To Target",
        createAlignToTargetCommand());

    NamedCommands.registerCommand(
        "Index On",
        Commands.runOnce(
            () -> indexer.runIndexers(
                IndexConstants.kIndexerStarSpeed,
                IndexConstants.kIndexerBackSpeed,
                IndexConstants.kIndexerAgitatorSpeed),
            indexer));

    NamedCommands.registerCommand(
        "Index Off",
        Commands.runOnce(indexer::stopAll, indexer));

    NamedCommands.registerCommand(
        "Waggle On",
        Commands.runOnce(intakeExtend::startOscillate, intakeExtend));

    NamedCommands.registerCommand(
        "Waggle Off",
        Commands.runOnce(intakeExtend::stopOscillateAndExtend, intakeExtend));
  }

  /* ===================== AUTON ===================== */
  private void configureAutonChooser() {
    autoChooser.setDefaultOption(kDefaultAuto, new PathPlannerAuto(kDefaultAuto));
    autoChooser.addOption(kAuto1, new PathPlannerAuto(kAuto1));
    autoChooser.addOption(kAuto2, new PathPlannerAuto(kAuto2));
    autoChooser.addOption(kAuto3, new PathPlannerAuto(kAuto3));
    autoChooser.addOption(kAuto4, new PathPlannerAuto(kAuto4));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /* ===================== SHUFFLEBOARD ===================== */
  private void buildShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Auton Select");
    tab.add("Auto Mode", autoChooser)
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .withPosition(0, 0)
        .withSize(2, 1);
  }

  /* ===================== LIFECYCLE PASSTHROUGHS ===================== */
  public void zeroIntakeAtStow() {
    intakeExtend.zeroAtStow();
  }

  public void setSwerveBrakeMode() {
    swerveSubsystem.setBrakeMode();
  }

  public void setSwerveCoastMode() {
    swerveSubsystem.setCoastMode();
  }
}