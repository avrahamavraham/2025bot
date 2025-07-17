package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Enums.ElevatorPosition;
import frc.robot.Enums.SwerveSpeedsLimits;
import frc.robot.constants.ElevatorSubsystemConstants;
import frc.robot.constants.ElevatorSubsystemConstants.MotionMagicConstants;
import frc.robot.constants.ElevatorSubsystemConstants.MotorCurrentLimits;
import frc.robot.constants.SwerveConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private final TalonFX m_master = new TalonFX(ElevatorSubsystemConstants.ELEVATOR_MASTER_MOTOR_ID, "canv");
  private final TalonFX m_slave = new TalonFX(ElevatorSubsystemConstants.ELEVATOR_SLAVE_MOTOR_ID, "canv");
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
  private DigitalInput m_closeSwitch;
  private boolean m_occurred = false;

  public ElevatorSubsystem() {
    m_closeSwitch = new DigitalInput(ElevatorSubsystemConstants.ELEVATOR_CLOSE_SWITCH_PORT);
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    CurrentLimitsConfigs limitConfigs = talonFXConfiguration.CurrentLimits;
    FeedbackConfigs feedbackConfigs = talonFXConfiguration.Feedback;
    feedbackConfigs.SensorToMechanismRatio = ElevatorSubsystemConstants.POSITION_CONVERSION_FACTOR;
    // feedbackConfigs.SensorToMechanismRatio = 1;

    limitConfigs.SupplyCurrentLimit = MotorCurrentLimits.SUPPLY_CURRENT_LIMIT;
    limitConfigs.SupplyCurrentLowerLimit = MotorCurrentLimits.SUPPLY_CURRENT_LOWER_LIMIT;
    limitConfigs.SupplyCurrentLimitEnable = MotorCurrentLimits.SUPPLY_CURRENT_LIMIT_ENABLE;

    MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = MotionMagicConstants.MOTION_MAGIC_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = MotionMagicConstants.MOTION_MAGIC_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = MotionMagicConstants.MOTION_MAGIC_JERK;

    Slot0Configs slot0 = talonFXConfiguration.Slot0;
    slot0.kS = MotionMagicConstants.MOTOR_KS;
    slot0.kG = MotionMagicConstants.MOTOR_KG;
    slot0.kV = MotionMagicConstants.MOTOR_KV;
    slot0.kA = MotionMagicConstants.MOTOR_KA;
    slot0.kP = MotionMagicConstants.MOTOR_KP;
    slot0.kI = MotionMagicConstants.MOTOR_KI;
    slot0.kD = MotionMagicConstants.MOTOR_KD;

    m_slave.setControl(new Follower(m_master.getDeviceID(), false));

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_master.getConfigurator().apply(talonFXConfiguration);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Is elevator down", isElevatorDown());
    SmartDashboard.putBoolean("Elevator first reset", m_occurred);
    SmartDashboard.putNumber("Elevator Height", getHeight());
    limitSwerveDriveSpeed();
    if (isElevatorDown()) {
      resetEncoder();
    }
    isFirstResetOccurred();
  }

  public void setLevel(double pos) {
    m_master.setControl(motionMagicVoltage.withPosition(pos).withSlot(0));
  }

  public double getVelocity() {
    return m_master.getVelocity().getValueAsDouble();
  }

  public double getHeight() {
    return m_master.getPosition().getValueAsDouble();
  }

  public boolean isElevatorDown() {
    return !m_closeSwitch.get();
  }

  public void stop() {
    m_master.set(0);
  }

  public void set(double speed) {
    m_master.set(speed);
  }

  public void resetEncoder() {
    m_master.setPosition(0);
  }

  public void limitSwerveDriveSpeed() {
    if (getHeight() > ElevatorPosition.L2.getTarget() && RobotState.isTeleop()) {
      SwerveConstants.SLOW_DRIVE = SwerveSpeedsLimits.halfSpeed.getTarget();
    } else {
      SwerveConstants.SLOW_DRIVE = SwerveSpeedsLimits.Normal.getTarget();
    }
  }

  public boolean isFirstResetOccurred() {
    if (isElevatorDown()) {
      m_occurred = true;
    }
    return m_occurred;
  }
}
