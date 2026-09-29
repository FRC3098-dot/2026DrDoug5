package frc.robot.util;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;


public final class Constants {

    /**
     * Constants that describe the per-module (individual swerve module) geometry and encoder
     * conversion factors.
     *
     * <p>Conversions below follow the pattern:
     * <ul>
     *   <li>distance per motor revolution = gearRatio * wheelCircumference (meters)</li>
     *   <li>angle per motor revolution = gearRatio * 2 * PI (radians)</li>
     * </ul>
     *
     * Notes:
     * - Gear ratio is expressed as motorRotations / outputRotations (so smaller numbers mean
     *   more motor rotations per output rotation). Verify that the ratios match your motor
     *   controller configuration (some libraries expect the inverse).
     */
    public static final class ModuleConstants {
        /** Wheel diameter used for linear distance conversions (meters). */
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3);

        /** Ratio from motor rotations to wheel rotations (motorRotations / wheelRotations). */
        public static final double kDriveMotorGearRatio = 1 / 5.08;

        /** Ratio from motor rotations to steering (turning) rotations. */
        public static final double kTurningMotorGearRatio = 1 / 46.42;

        /**
         * Converts a drive-motor rotation to meters traveled by the wheel.
         * Calculation: motorRot * gearRatio * circumference
         */
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;

        /** Converts a turning-motor rotation to radians of steering rotation. */
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;

        /** RPM -> meters / second for drive encoder. */
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;

        /** RPM -> radians / second for turning encoder. */
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

        /** Proportional gain used by the turning PID loop. Tune if modules oscillate or are slow. */
        public static final double kPTurning = 0.15;
    }

    /**
     * Drive subsystem physical and tuning constants.
     */
    public static final class DriveConstants {
        /** Distance between left and right wheels (meters). Used by kinematics. */
        public static final double kTrackWidth = Units.inchesToMeters(24);

        /** Distance between front and back wheels (meters). Used by kinematics. */
        public static final double kWheelBase = Units.inchesToMeters(24);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front Left
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front Right
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back Left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Back Right

        
// NAMING CONVENTION:
// F = Front (Front Side of Robot)
// B = Back (Back Side of Robot)
// L = Left (Driver Side) - Driver Station perspective, facing the field
// R = Right  (Passenger Side) - Driver Station perspective, facing the field
// D = Drive (wheel rotation / translation)
// T = Turn  (module steering / azimuth)
// 

// Examples:
// FLD = Front Left Drive
// FLT = Front Left Turn
//
// Absolute encoders are TURN encoders.
// Offsets are stored as ROTATION FRACTIONS (0–1), not radians.


        public static final int kFLDMotorPort = 8;
        public static final int kBLDMotorPort = 2;
        public static final int kFRDMotorPort = 6;
        public static final int kBRDMotorPort = 4;

        public static final int kFLTMotorPort = 7;
        public static final int kBLTMotorPort = 1;
        public static final int kFRTMotorPort = 5;
        public static final int kBRTMotorPort = 3;

        public static final boolean kFLTEncReversed = false;
        public static final boolean kBLTEncReversed = false;
        public static final boolean kFRTEncReversed = false;
        public static final boolean kBRTEncReversed = false;

        public static final boolean kFLDEncReversed = false;
        public static final boolean kBLDEncReversed = false;
        public static final boolean kFRDEncReversed = false;
        public static final boolean kBRDEncReversed = false;

        public static final int kFLTAbsEncPort = 0;
        public static final int kBLTAbsEncPort = 2;
        public static final int kFRTAbsEncPort = 1;
        public static final int kBRTAbsEncPort = 3;

        public static final boolean kFLTAbsEncReversed = true;
        public static final boolean kBLTAbsEncReversed = true;
        public static final boolean kFRTAbsEncReversed = true;
        public static final boolean kBRTAbsEncReversed = true;

       /** This is the Raw angle devided 360 */
        public static final double kFLTAbsEncOffsetRot = 0.360; //.355
        public static final double kBLTAbsEncOffsetRot = 0.372;//.358
        public static final double kFRTAbsEncOffsetRot = 0.025;//.977
        public static final double kBRTAbsEncOffsetRot =  0.027; //.995

        //Current Limits
        public static final int kDriveMotorCurrLim = 40;
        public static final int kTurnMotorCurrLim = 20;


        /** Theoretical maximum linear speed of the robot (m/s). Use conservatively. */
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;

        /** Theoretical maximum angular speed (rad/s). 4 * PI corresponds to 2 full rotations/sec. */
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        /** Deadband applied to joystick translation axis (unitless, 0-1). Helps ignore small stick noise. */
        public static double kControllerDeadband = 0.05;

        /** Deadband applied to joystick rotation axis (unitless, 0-1). */
        public static double kControllerRotDeadband = 0.05;
    }

    /**
     * Autonomous driving and path-following tuning constants. These values are used by the
     * trajectory follower and nested controllers (PID / feedforward). Reduce them conservatively
     * when testing on the real robot.
     */
    public static final class AutoConstants {
        /** Max translation speed used while generating autonomous trajectories (m/s). */
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;


        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);

        /** PID constants for translation controller (PathPlanner / Trajectory following). */
        public static final PIDConstants translationConstants = new PIDConstants(17.0, 0.05, 0.0);

        /** PID constants for rotation controller used during path following. */
        public static final PIDConstants rotationConstants = new PIDConstants(10.0, 0.05, 0.0);
    }

    /**
     * Operator Interface constants: controller ports, axis indices and button mappings.
     *
     * <p>Button and axis numbering follows the controller library used (typically WPILib/Xbox
     * mapping). If you swap controllers, verify these indices match the new device.
     */
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kCoDriverControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 5; // Lt Bumper
        public static final int kdriverBack = 7;

        public static final int kCodriver_B = 2;
        public static final int kCodriver_Y = 4;
        public static final int kCodriver_A = 1;
        public static final int kCodriver_X = 3;

        public static final int kCodriverRTrigger = 3;
        public static final int kCodriverLTrigger = 2;
        public static final int kCodriverRBumper = 6;
        public static final int kCodriverLBumper = 5;
        public static final int kCodriverBack = 7;

        public static final double kCodriverLJoystickY = 1;
        public static final double kCodriverRJoystickY = 5;
        public static final double kDeadband = 0.05;

// ==================== D-PAD BUMP DRIVING ====================
        // Robot-centric "nudge" moves using D-pad + RT.
        // These ONLY affect D-pad bumps, NOT normal joystick driving.

        /** Minimum bump speed when RT is lightly pressed (m/s). */
        public static final double kBumpMinSpeedMps = 1.0;

        /** Maximum bump speed when RT is fully pressed (m/s). */
        public static final double kBumpMaxSpeedMps = 3.0;

        /** Deadband for right trigger to ignore noise. */
        public static final double kRtDeadband = 0.05;


    }

public static final class IntakeExtendConstants {

  /* ===================== Hardware ===================== */

  public static final int kIntakeExtendMotor = 21; // example ID
  public static final boolean kIntakeExtendInverted = false;
  public static final int kIntakeExtendCurrentLimit = 40;

  // Encoder conversion: degrees per motor revolution
  public static final double kArmDegPerMotorRev = 14.4; // example

  /* ===================== Arm Angles ===================== */

  // Mechanical limits
  public static final double kMinAngleDeg = 0.0;
  public static final double kMaxAngleDeg = 120.0;

  // Preset positions
  public static final double kAutonExtendAngleDeg = 85.0;

  /* ===================== PID ===================== */

  public static final double kArmPosP = 0.03;
  public static final double kArmPosI = 0.0;
  public static final double kArmPosD = 0.00;

  public static final double kArmToleranceDeg = 5.0;

  /* ===================== Motion Profile ===================== */

  // Base motion limits (deg/sec, deg/sec^2)
  public static final double kMaxVelDegPerSec = 90.0;
  public static final double kMaxAccelDegPerSec2 = 180.0;

  /* =========================================================
   * Gravity / Motion Shaping
   * ========================================================= */

  // Gravity-neutral angle:
  // Below this -> gravity assists retract
  // Above this -> gravity assists extend
  public static final double kGravityNeutralDeg = 15.0;

  // Degrees above stow where output is softened
  public static final double kStowSoftZoneDeg = 10.0;

  // Output limit when gravity is helping motion
  public static final double kGravityAssistOutputLimit = 0.25;

  // Output limit near the stow hard stop
  public static final double kNearStowOutputLimit = 0.25;

  // Scale applied to velocity & acceleration when gravity helps.
  // 0.20 ≈ ~3–4 seconds per full waggle cycle (very gentle on motor)
  public static final double kGravityAssistConstraintScale = 0.35;

  /* =========================================================
   * Intake Oscillation (Waggle)
   * ========================================================= */

  // Waggle bounds
  public static final double kOscillateMinDeg = 10.0;
  public static final double kOscillateMaxDeg = 50.0;

  // How close to the end before switching direction
  public static final double kOscillateToleranceDeg = 2.0;

  // Pause time at each end of waggle (seconds)
  // Slightly increased to further reduce reversals & motor stress
  public static final double kOscillateDwellSec = 0.20;
}
    /** Shooter subsystem constants: flywheels motor IDs and speeds. */
    public static final class ShooterConstants {
    
        // CAN IDs - update to match your robot wiring
        public static final int kBackShooterMotorID = 16;
        public static final int kFrontShooterMotorID = 17;

        // Inverted?
        public static final boolean kBackShooterMotorInverted = false;
        public static final boolean kFrontShooterMotorInverted = true;
        
        //Current Limits
        public static final int kBackShooterCurrLim = 40;
        public static final int kFrontShooterCurrLim = 40;


        // Default speeds
        public static final double kFlywheelFrontFarSpeed = 0.6; 
        public static final double kFlywheelBackFarSpeed = 0.10; 
        public static final double kFlywheelFrontCloseSpeed = 0.40; 
        public static final double kFlywheelBackCloseSpeed = 0.10; 
        
        // PASS shot speeds (tune on robot)
        public static final double kFlywheelFrontPassSpeed = 0.80;
        public static final double kFlywheelBackPassSpeed  = 0.80;

        // Axis threshold for considering trigger "pressed"
        public static final double kTriggerThreshold = 0.1;
    }
public static final class IndexConstants {
        // CAN IDs - update to match your robot wiring
    public static final int kIndexerStarMotorID = 18;
    public static final int kIndexerBackMotorID = 20;
    public static final int kIndexerAgitatorMotorID = 15;

    // Inverted?
    public static final boolean kIndexerStarMotorInverted = false;
    public static final boolean kIndexerBackMotorInverted = true;
    public static final boolean kIndexerAgitatorMotorInverted = false;

    // Current Limits
    public static final int kIndexerStarCurrLim = 40;
    public static final int kIndexerBackCurrLim = 40;
    public static final int kIndexerAgitatorCurrLim = 20;

    // Default speeds
    public static final double kIndexerStarSpeed = 0.5;
    public static final double kIndexerBackSpeed = 0.8;
    public static final double kIndexerAgitatorSpeed = 0.1;


    }
    // ---------------- Fuel / Arm constants ----------------       
    /**
     * Fuel/arm subsystem constants. Includes CAN IDs, limit switches, gear geometry, and PID
     * / feedforward values for the arm mechanism.
     */
    public static final class FuelConstants {
        // CAN IDs
        public static final int kFuelIntakeMotorID = 13;

    // Current Limits
        public static final boolean kFuelIntakeInverted = true;

    // Default speeds
        public static final int kFuelIntakeCurrLim = 20;
    }

 // ---------------- Vision constants ----------------     
    public static final class VisionConstants {
    /** Primary (front) camera name used by most subsystems. */
    public static final String CameraFront = "limelight-frtcam";
    /** Secondary (back) camera name. */
    public static final String CameraBack = "limelight-bckcam";
    // Backwards-compatible alias used by older code
    public static final String CameraName = CameraFront; // default camera name
    }
}
