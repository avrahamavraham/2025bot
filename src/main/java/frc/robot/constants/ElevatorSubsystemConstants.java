package frc.robot.constants;

public class ElevatorSubsystemConstants {

  public static final int ELEVATOR_CLOSE_SWITCH_PORT = 2;
  public static final int ELEVATOR_MASTER_MOTOR_ID = 40;
  public static final int ELEVATOR_SLAVE_MOTOR_ID = 41;

  public static final double POSITION_CONVERSION_FACTOR = 48.7;

  // public static final double POSITION_CONVERSION_FACTOR = 1;

  public static final class MotorCurrentLimits {

    public static final int SUPPLY_CURRENT_LIMIT = 30;
    public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 30;
  }

  public static final class MotionMagicConstants {

    public static final double MOTION_MAGIC_VELOCITY = 2;
    public static final double MOTION_MAGIC_ACCELERATION = 2.5;
    public static final double MOTION_MAGIC_JERK = 25;

    public static final double MOTOR_KS = 0.045; // TODO INITILIZE THESE VALUES
    public static final double MOTOR_KA = 0.5;
    public static final double MOTOR_KV = 6.5;
    public static final double MOTOR_KG = 0.015;
    public static final double MOTOR_KP = 60;
    public static final double MOTOR_KI = 0;
    public static final double MOTOR_KD = 0;
  }
}
