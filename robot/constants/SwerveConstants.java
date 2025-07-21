package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public class SwerveConstants {

  public static final COTSTalonFXSwerveConstants CHOSEN_MODULE = // TODO: This must be tuned to specific robot
    COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);
  public static final boolean INVERT_GYRO = true;
  public static double SLOW_DRIVE = 1;

  /* Drivetrain Constants */
  public static final double TRACK_WIDTH = 0.577; // TODO: This must be tuned to specific robot
  public static final double WHEEL_BASE = 0.666; // TODO: This must be tuned to specific robot
  public static final double WHEEL_CIRCUMFERENCE = CHOSEN_MODULE.wheelCircumference;

  /*
   * Swerve Kinematics
   * No need to ever change this unless you are not doing a traditional
   * rectangular/square 4 module swerve
   */
  public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
    new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
    new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
    new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
    new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
  );

  /* Module Gear Ratios */
  public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
  public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

  /* Motor Inverts */
  public static final InvertedValue ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
  public static final InvertedValue DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

  /* Angle Encoder Invert */
  public static final SensorDirectionValue CANCODER_INVERT = CHOSEN_MODULE.cancoderInvert;

  /* Swerve Current Limiting */
  public static final int ANGLE_CURRENT_LOWER_LIMIT = 25;
  public static final int ANGLE_CURRENT_THRESHOLD = 40;
  public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
  public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

  public static final int DRIVE_CURRENT_LOWER_LIMIT = 35;
  public static final int DRIVE_CURRENT_THRESHOLD = 60;
  public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
  public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

  /*
   * These values are used by the drive falcon to ramp in open loop and closed
   * loop driving.
   * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
   */
  public static final double OPEN_LOOP_RAMP = 0.25;
  public static final double CLOSED_LOOP_RAMP = 0.0;

  /* Angle Motor PID Values */
  public static final double ANGLE_KP = CHOSEN_MODULE.angleKP;
  public static final double ANGLE_KI = CHOSEN_MODULE.angleKI;
  public static final double ANGLE_KD = CHOSEN_MODULE.angleKD;

  /* Drive Motor PID Values */
  public static final double DRIVE_KP = 0.12; // TODO: This must be tuned to specific robot
  public static final double DRIVE_KI = 0.0;
  public static final double DRIVE_KD = 0.0;
  public static final double DRIVE_KF = 0.0;

  /* Drive Motor Characterization Values From SYSID */
  public static final double DRIVE_KS = 0.32; // TODO: This must be tuned to specific robot
  public static final double DRIVE_KV = 1.51;
  public static final double DRIVE_KA = 0.27;

  /* Swerve Profiling Values */
  /** Meters per Second */
  // public static final double maxSpeed = 4.5; //TODO: This must be tuned to
  // specific robot
  public static final double MAX_SPEED = 5.03; // TODO: This must be tuned to specific robot
  /** Radians per Second */
  public static final double MAX_ANGULAR_VELOCITY = 15.0; // TODO: This must be tuned to specific robot

  /* Neutral Modes */
  public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
  public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
  public static final double ANGLE_CURRENT_LIMIT = 0;
  public static final double DRIVE_CURRENT_LIMIT = 0;

  /* Module Specific Constants */
  /* Front Left Module - Module 0 */
  public static final class Mod0 { // TODO: This must be tuned to specific robot

    public static final int DRIVE_MOTOR_ID = 1;
    public static final int ANGLE_MOTOR_ID = 2;
    public static final int CAN_CODER_ID = 1;
    public static final Rotation2d ANGLE_OFFSET = new Rotation2d(Units.rotationsToRadians(0.031982));
    public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
      DRIVE_MOTOR_ID,
      ANGLE_MOTOR_ID,
      CAN_CODER_ID,
      ANGLE_OFFSET
    );
  }

  /* Front Right Module - Module 1 */
  public static final class Mod1 { // TODO: This must be tuned to specific robot

    public static final int DRIVE_MOTOR_ID = 3;
    public static final int ANGLE_MOTOR_ID = 4;
    public static final int CAN_CODER_ID = 2;
    public static final Rotation2d ANGLE_OFFSET = new Rotation2d(Units.rotationsToRadians(0.445068));
    public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
      DRIVE_MOTOR_ID,
      ANGLE_MOTOR_ID,
      CAN_CODER_ID,
      ANGLE_OFFSET
    );
  }

  /* Back Left Module - Module 2 */
  public static final class Mod2 { // TODO: This must be tuned to specific robot

    public static final int DRIVE_MOTOR_ID = 5;
    public static final int ANGLE_MOTOR_ID = 6;
    public static final int CAN_CODER_ID = 3;
    public static final Rotation2d ANGLE_OFFSET = new Rotation2d(Units.rotationsToRadians(0.000732));
    public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
      DRIVE_MOTOR_ID,
      ANGLE_MOTOR_ID,
      CAN_CODER_ID,
      ANGLE_OFFSET
    );
  }

  /* Back Right Module - Module 3 */
  public static final class Mod3 { // TODO: This must be tuned to specific robot

    public static final int DRIVE_MOTOR_ID = 7;
    public static final int ANGLE_MOTOR_ID = 8;
    public static final int CAN_CODER_ID = 4;
    public static final Rotation2d ANGLE_OFFSET = new Rotation2d(Units.rotationsToRadians(0.166748));
    public static final SwerveModuleConstants CONSTANTS = new SwerveModuleConstants(
      DRIVE_MOTOR_ID,
      ANGLE_MOTOR_ID,
      CAN_CODER_ID,
      ANGLE_OFFSET
    );
  }
}
