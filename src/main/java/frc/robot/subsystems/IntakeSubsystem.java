package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeSubsystemConstants;
import frc.robot.constants.IntakeSubsystemConstants.MotorCurrentLimits;

public class IntakeSubsystem extends SubsystemBase {

  private SparkMax m_angleMotor;
  private SparkFlex m_wheelMotor;

  private SparkMaxConfig m_config;
  private SparkFlexConfig m_configSparkFlex;
  private SparkClosedLoopController m_controller;
  private SparkClosedLoopController m_angleController;

  private DigitalInput m_closeSwitchLeft;
  private DigitalInput m_closeSwitchRight;

  private boolean occurred;

  public IntakeSubsystem() {
    occurred = false;
    m_closeSwitchLeft = new DigitalInput(IntakeSubsystemConstants.INTAKE_LEFT_SENSOR_PORT);
    m_closeSwitchRight = new DigitalInput(IntakeSubsystemConstants.INTAKE_RIGHT_SENSOR_PORT);

    m_angleMotor = new SparkMax(IntakeSubsystemConstants.ALGAE_INTAKE_ANGLE_MOTOR_ID, MotorType.kBrushless);
    m_wheelMotor = new SparkFlex(IntakeSubsystemConstants.ALGAE_INTAKE_WHEEL_MOTOR_ID, MotorType.kBrushless);
    m_config = new SparkMaxConfig();
    m_configSparkFlex = new SparkFlexConfig();

    m_configSparkFlex.inverted(true).idleMode(IdleMode.kCoast);
    m_configSparkFlex.encoder.velocityConversionFactor(IntakeSubsystemConstants.VELOCITY_CONVERSION_FACTOR);
    m_configSparkFlex.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(IntakeSubsystemConstants.WHEEL_KP, IntakeSubsystemConstants.WHEEL_KI, IntakeSubsystemConstants.WHEEL_KD)
      .velocityFF(IntakeSubsystemConstants.WHEEL_KFF);
    m_configSparkFlex.smartCurrentLimit(MotorCurrentLimits.SUPPLY_CURRENT_LIMIT);
    m_controller = m_wheelMotor.getClosedLoopController();
    m_wheelMotor.configure(m_configSparkFlex, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_config.inverted(true).idleMode(IdleMode.kBrake);
    m_config.encoder.positionConversionFactor(IntakeSubsystemConstants.POSITION_CONVERSION_FACTOR);
    m_config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(IntakeSubsystemConstants.MOTOR_KP, IntakeSubsystemConstants.MOTOR_KI, IntakeSubsystemConstants.MOTOR_KD);
    // m_config.closedLoop.maxMotion
    //   .maxAcceleration(IntakeSubsystemConstants.MAX_ACCELERATION)
    //   .maxVelocity(IntakeSubsystemConstants.MAX_VELOCITY);
    m_config.smartCurrentLimit(MotorCurrentLimits.SUPPLY_CURRENT_LIMIT_ANGLE);
    m_angleController = m_angleMotor.getClosedLoopController();
    m_angleMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Is intake left close", isIntakeLeftClose());
    SmartDashboard.putBoolean("Is intake right close", isIntakeRightClose());
    SmartDashboard.putBoolean("Intake first reset", occurred);
    SmartDashboard.putNumber("intake pos", getAngle());
    SmartDashboard.putNumber("intake vel", getWheelsVelocity());
    SmartDashboard.putNumber("intake angle output", m_wheelMotor.get());
    SmartDashboard.putNumber("intake wheels voltage", m_wheelMotor.getOutputCurrent());
    if (isIntakeRightClose() || isIntakeLeftClose()) {
      m_angleMotor.getEncoder().setPosition(0);
    }
    isFirstResetOccurred();
  }

  public void setWheelsVelocity(double targetVelocityRPM) {
    // m_controller.setReference(targetVelocityRPM, ControlType.kVelocity);
    m_wheelMotor.set(targetVelocityRPM);
  }

  public void set(double speed) {
    m_wheelMotor.set(speed);
  }

  public void stop() {
    m_wheelMotor.set(0);
  }

  public double getWheelsVelocity() {
    return m_wheelMotor.getEncoder().getVelocity();
  }

  public boolean isIntakeLeftClose() {
    return !m_closeSwitchLeft.get();
  }

  public boolean isIntakeRightClose() {
    return !m_closeSwitchRight.get();
  }

  public double getAngleOutput() {
    return m_angleMotor.get();
  }

  public void setAngleOutput(double output) {
    m_angleMotor.set(output);
  }

  public void setAnglePosition(double angle) {
    m_angleController.setReference(angle, ControlType.kPosition);
  }

  public double getAngle() {
    return m_angleMotor.getEncoder().getPosition();
  }

  public void resetAngle() {
    m_angleMotor.getEncoder().setPosition(0);
  }

  public boolean isFirstResetOccurred() {
    if (isIntakeRightClose() || isIntakeLeftClose()) {
      occurred = true;
    }
    return occurred;
  }
}
