package frc.robot.constants;

public class ClimbSubsystemConstants {

  public static final class MotorCurrentLimits {

    public static final int SUPPLY_CURRENT_LIMIT = 40;
    public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 0;
  }

  public static final int CLIMB_MOTOR_ID = 60;
  public static final int POSITION_CONVERSION_FACTOR = 1;
  public static final int VELOCITY_CONVERSION_FACTOR = 1000;

  public static final int WHEEL_KP = -0;
  public static final int WHEEL_KI = -0;
  public static final int WHEEL_KD = -0;
  public static final int WHEEL_KFF = -0;

  public static final class MotionMagicConstants {

    public static final double MOTION_MAGIC_VELOCITY = -0;
    public static final double MOTION_MAGIC_ACCELERATION = -0;
    public static final double MOTION_MAGIC_JERK = -0;

    public static final int MOTOR_KS = -0; // TODO INITILIZE THESE VALUES
    public static final int MOTOR_KA = -0;
    public static final int MOTOR_KV = -0;
    public static final int MOTOR_KP = -0;
    public static final int MOTOR_KI = -0;
    public static final int MOTOR_KD = -0;
  }
}
