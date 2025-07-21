package frc.robot.constants;

public class IntakeSubsystemConstants {

  public static final class MotorCurrentLimits {

    public static final int SUPPLY_CURRENT_LIMIT = 35;
    public static final int SUPPLY_CURRENT_LIMIT_ANGLE = 20;
  }

  public static final int ALGAE_INTAKE_ANGLE_MOTOR_ID = 50;
  public static final int ALGAE_INTAKE_WHEEL_MOTOR_ID = 51;
  public static final int INTAKE_LEFT_SENSOR_PORT = 7;
  public static final int INTAKE_RIGHT_SENSOR_PORT = 8;

  // public static final double POSITION_CONVERSION_FACTOR = 0.002777777777777778; // 1 / 360\][]
  public static final double POSITION_CONVERSION_FACTOR = 10.470879716491785;
  public static final double VELOCITY_CONVERSION_FACTOR = 1;

  public static final double WHEEL_KP = 0.4300;
  public static final double WHEEL_KI = -0;
  public static final double WHEEL_KD = 0.0015;
  public static final double WHEEL_KFF = 0.0001;

  public static final double MOTOR_KP = 0.008;
  public static final double MOTOR_KI = 0;
  public static final double MOTOR_KD = 0;
  public static final double MAX_VELOCITY = 6000;
  public static final double MAX_ACCELERATION = 6000;
}
