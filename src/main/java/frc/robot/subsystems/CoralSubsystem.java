package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralSubsystemConstants;
import frc.robot.constants.CoralSubsystemConstants.MotionMagicConstants;
import frc.robot.constants.CoralSubsystemConstants.MotorCurrentLimits;

public class CoralSubsystem extends SubsystemBase {

  private TalonFX m_angleMotor;
  private SparkFlex m_intakeMotor;

  private DigitalInput m_closeSwitch;
  private DigitalInput m_coralDetector;

  private SparkFlexConfig m_config;
  private StatusSignal<Angle> m_angleStatusSignal;
  private SparkClosedLoopController m_controller;
  private PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  private boolean occurred;
  private boolean startTimer = false;
  private boolean checkTimer = false;
  private Timer t = new Timer();

  public CoralSubsystem() {
    m_coralDetector = new DigitalInput(CoralSubsystemConstants.CORAL_IN_SENSOR_PORT);
    m_closeSwitch = new DigitalInput(CoralSubsystemConstants.CORAL_CLOSE_SENSOR_PORT);

    m_angleMotor = new TalonFX(CoralSubsystemConstants.ANGLE_MOTOR_ID, "rio");
    m_intakeMotor = new SparkFlex(CoralSubsystemConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    m_config = new SparkFlexConfig();
    m_controller = m_intakeMotor.getClosedLoopController();
    m_angleStatusSignal = m_angleMotor.getPosition();

    m_config
      .inverted(CoralSubsystemConstants.WHEELS_MOTOR_IS_INVERTED)
      .idleMode(CoralSubsystemConstants.WHEELS_MOTOR_IDLE_MODE);
    m_config.encoder
      .positionConversionFactor(CoralSubsystemConstants.INTAKE_POSITION_CONVERSION_FACTOR)
      .velocityConversionFactor(CoralSubsystemConstants.VELOCITY_CONVERSION_FACTOR);
    m_config.closedLoop
      .feedbackSensor(CoralSubsystemConstants.WHEELS_MOTOR_SENSOR)
      .pid(CoralSubsystemConstants.WHEELS_KP, CoralSubsystemConstants.WHEELS_KI, CoralSubsystemConstants.WHEELS_KD)
      .velocityFF(CoralSubsystemConstants.WHEELS_KFF);
    m_config.smartCurrentLimit(MotorCurrentLimits.SUPPLY_CURRENT_LIMIT_WHEELS);

    m_intakeMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    occurred = false;

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    CurrentLimitsConfigs limitConfigs = cfg.CurrentLimits;

    DutyCycleOut a = new DutyCycleOut(0.5).withLimitForwardMotion(true).withLimitReverseMotion(true);

    cfg.MotorOutput.Inverted = CoralSubsystemConstants.ENCODER_INVERTED;
    /* Configure gear ratio */
    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = CoralSubsystemConstants.ANGLE_POSITION_CONVERSION_FACTOR;

    limitConfigs.SupplyCurrentLimit = MotorCurrentLimits.SUPPLY_CURRENT_LIMIT_ANGLE;
    limitConfigs.SupplyCurrentLowerLimit = MotorCurrentLimits.SUPPLY_CURRENT_LOWER_LIMIT;
    limitConfigs.SupplyCurrentLimitEnable = MotorCurrentLimits.SUPPLY_CURRENT_LIMIT_ENABLE;
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    MotionMagicConfigs motionMagicConfigs = cfg.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = MotionMagicConstants.MOTION_MAGIC_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = MotionMagicConstants.MOTION_MAGIC_ACCELERATION;
    motionMagicConfigs.MotionMagicJerk = MotionMagicConstants.MOTION_MAGIC_JERK;

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kS = MotionMagicConstants.KS;
    slot0.kG = MotionMagicConstants.KG;
    slot0.kV = MotionMagicConstants.KV;
    slot0.kA = MotionMagicConstants.KA;
    slot0.kP = MotionMagicConstants.KP;
    slot0.kI = MotionMagicConstants.KI;
    slot0.kD = MotionMagicConstants.KD;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_angleMotor.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Is Coral System Close", isCoralSystemClose());
    SmartDashboard.putBoolean("Is coral in", isCoralIn());
    SmartDashboard.putBoolean("Is coral out", isCoralOut());
    SmartDashboard.putBoolean("Coral first reset", occurred);
    SmartDashboard.putNumber("Coral Pos", getAngle());
    SmartDashboard.putNumber("Coral output", getAngleOutput());
    SmartDashboard.putNumber("Coral velocity", getAngleVelocity());
    SmartDashboard.putNumber("Coral angle  voltage", m_angleMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Coral intake  voltage", m_intakeMotor.getOutputCurrent());

    if (isCoralSystemClose()) {
      resetAngle();
    }
    isFirstResetOccurred();
  }

  public boolean isCoralSystemClose() {
    return !m_closeSwitch.get();
  }

  public void setAngle(double level) {
    m_angleMotor.setControl(motionMagicVoltage.withPosition(level).withSlot(0));
  }

  public double getAngle() {
    return m_angleMotor.getPosition().getValueAsDouble();
  }

  public void setOutput(double output) {
    m_angleMotor.set(output);
  }

  public void stop() {
    m_angleMotor.set(0);
  }

  public double getHeight() {
    return m_angleStatusSignal.getValueAsDouble();
  }

  public boolean isCoralIn() {
    //  if output == 0
    //    return last status
    //  else if output && velocity
    //    if timer not running
    //      start timer
    //    else if time pass
    //      return true
    //  else
    //    stop timer
    //  return false

    boolean lastStatus = false;
    if (getOutput() == 0) {
      return lastStatus;
    } else if (getOutput() < 0 && getVelocity() > CoralSubsystemConstants.CORAL_IN_VELOCITY) {
      if (!t.isRunning()) {
        t.restart();
      } else if (t.get() > CoralSubsystemConstants.REQ_TIME) {
        return true;
      }
    } else {
      t.stop();
    }
    return false;
    // if (getOutput() < 0 && getVelocity() > CoralSubsystemConstants.CORAL_IN_VELOCITY) {
    //   if (!startTimer) {
    //     startTimer = true;
    //     checkTimer = true;
    //     t.start();
    //   }
    // }
    // if (checkTimer) {
    //   if (getTime() > CoralSubsystemConstants.REQ_TIME && getVelocity() > CoralSubsystemConstants.CORAL_IN_VELOCITY) {
    //     startTimer = false;
    //     checkTimer = false;
    //     t.reset();
    //     return true;
    //   }
    // }
    // return false;
  }

  public boolean isCoralOut() {
    boolean lastStatus = false;
    if (getOutput() == 0) {
      return lastStatus;
    } else if (getOutput() > 0 && getVelocity() < CoralSubsystemConstants.CORAL_IN_VELOCITY) {
      if (!t.isRunning()) {
        t.restart();
      } else if (t.get() < CoralSubsystemConstants.REQ_TIME) {
        return true;
      }
    } else {
      t.stop();
    }
    return false;
    // if (getOutput() > 0 && getVelocity() < CoralSubsystemConstants.CORAL_OUT_VELOCITY) {
    //   if (!startTimer) {
    //     startTimer = true;
    //     checkTimer = true;
    //     t.start();
    //   }
    // }
    // if (checkTimer) {
    //   if (getTime() > CoralSubsystemConstants.REQ_TIME && getVelocity() < CoralSubsystemConstants.CORAL_OUT_VELOCITY) {
    //     startTimer = false;
    //     checkTimer = false;
    //     t.reset();
    //     return true;
    //   }
    // }
    // return false;
  }

  public double getVelocity() {
    return m_intakeMotor.get();
  }

  public double getOutput() {
    return m_intakeMotor.get();
  }

  public double getAngleOutput() {
    return m_angleMotor.get();
  }

  public double getAngleVelocity() {
    return m_angleMotor.getVelocity().getValueAsDouble();
  }

  public double getTime() {
    return t.get();
  }

  public void setIntakeOutput(double output) {
    m_intakeMotor.set(output);
  }

  public void setVelocity(double velocity) {
    // m_controller.setReference(velocity, ControlType.kVelocity);
    m_intakeMotor.set(velocity);
  }

  public void resetAngle() {
    m_angleMotor.setPosition(0);
  }

  public boolean isFirstResetOccurred() {
    if (isCoralSystemClose()) {
      occurred = true;
    }
    return occurred;
  }
}
