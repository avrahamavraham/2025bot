package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SetSubsystemTargetCommand;
import frc.robot.constants.ClimbSubsystemConstants;
import frc.robot.constants.ClimbSubsystemConstants.MotionMagicConstants;
import frc.robot.constants.ClimbSubsystemConstants.MotorCurrentLimits;
import java.util.function.DoubleSupplier;

public class ClimbSubsystem extends SubsystemBase {

  private TalonFX m_crankMotor;
  private PositionDutyCycle m_positionDutyCycle;

  public ClimbSubsystem() {
    m_crankMotor = new TalonFX(ClimbSubsystemConstants.CLIMB_MOTOR_ID, "canv");

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    m_positionDutyCycle = new PositionDutyCycle(0);

    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kS = MotionMagicConstants.MOTOR_KS;
    slot0Configs.kV = MotionMagicConstants.MOTOR_KV;
    slot0Configs.kA = MotionMagicConstants.MOTOR_KA;

    slot0Configs.kP = MotionMagicConstants.MOTOR_KP;
    slot0Configs.kI = MotionMagicConstants.MOTOR_KI;
    slot0Configs.kD = MotionMagicConstants.MOTOR_KD;

    limitConfigs.SupplyCurrentLimit = MotorCurrentLimits.SUPPLY_CURRENT_LIMIT;
    limitConfigs.SupplyCurrentLowerLimit = MotorCurrentLimits.SUPPLY_CURRENT_LOWER_LIMIT;
    limitConfigs.SupplyCurrentLimitEnable = MotorCurrentLimits.SUPPLY_CURRENT_LIMIT_ENABLE;

    motionMagicConfigs.MotionMagicCruiseVelocity = MotionMagicConstants.MOTION_MAGIC_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = MotionMagicConstants.MOTION_MAGIC_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = MotionMagicConstants.MOTION_MAGIC_JERK;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    feedbackConfigs.withSensorToMechanismRatio(ClimbSubsystemConstants.POSITION_CONVERSION_FACTOR);

    m_crankMotor.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("climb voltage", m_crankMotor.getMotorVoltage().getValueAsDouble());
  }

  public void setOutput(DoubleSupplier output) {
    m_crankMotor.set(output.getAsDouble());
  }

  public void setPosition(double angle) {
    m_crankMotor.setControl(m_positionDutyCycle.withPosition(angle));
  }

  public double getPosition() {
    return m_crankMotor.getPosition().getValueAsDouble();
  }
}
