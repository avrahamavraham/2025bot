package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class CoralSubsystemConstants {

  public static final int CORAL_IN_SENSOR_PORT = 5;
  public static final int CORAL_CLOSE_SENSOR_PORT = 3;

  public static final class MotorCurrentLimits {

    public static final int SUPPLY_CURRENT_LIMIT_ANGLE = 20;
    public static final int SUPPLY_CURRENT_LIMIT_WHEELS = 35;
    public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 10;
  }

  public static final boolean WHEELS_MOTOR_IS_INVERTED = true;
  public static final IdleMode WHEELS_MOTOR_IDLE_MODE = IdleMode.kBrake;
  public static final FeedbackSensor WHEELS_MOTOR_SENSOR = FeedbackSensor.kPrimaryEncoder;
  public static final int INTAKE_MOTOR_ID = 30;
  public static final int ANGLE_MOTOR_ID = 31;

  public static final double ANGLE_POSITION_CONVERSION_FACTOR = 1 / 3.5849460274239036; // 25.10498046875 / 90
  public static final double INTAKE_POSITION_CONVERSION_FACTOR = 1000; // TODO UPDATE
  public static final int VELOCITY_CONVERSION_FACTOR = 1000; // TODO UPDATE

  public static final InvertedValue ENCODER_INVERTED = InvertedValue.Clockwise_Positive;

  public static final double WHEELS_KP = 0;
  public static final double WHEELS_KI = 0;
  public static final double WHEELS_KD = 0;
  public static final double WHEELS_KFF = 0;
  public static final double ANGLE_THRESHOLD = 0;
  public static final double CORAL_IN_VELOCITY = 0;
  public static final double REQ_TIME = 0;
  public static final double CORAL_OUT_VELOCITY = 0;

  public static final class MotionMagicConstants {

    public static final double MOTION_MAGIC_VELOCITY = 400;
    public static final double MOTION_MAGIC_ACCELERATION = 1000;
    public static final double MOTION_MAGIC_JERK = 100000;

    public static final double KS = 0.012;
    public static final double KG = 0;
    public static final double KV = 0.032;
    public static final double KA = 0.0009;
    public static final double KP = 1;
    public static final double KI = 0;
    public static final double KD = 0;
  }
}
