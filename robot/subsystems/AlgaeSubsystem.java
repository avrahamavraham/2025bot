package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeSubsystemConstants;

public class AlgaeSubsystem extends SubsystemBase {

  private DigitalInput m_algaeDetector;

  private TalonFX m_holderMotor;
  private VelocityDutyCycle m_velocityDutyCycle;

  private boolean startTimer = false;
  private boolean checkTimer = false;
  private Timer timer = new Timer();

  public AlgaeSubsystem() {
    m_algaeDetector = new DigitalInput(AlgaeSubsystemConstants.ALGAE_SENSOR_PORT);
    m_holderMotor = new TalonFX(AlgaeSubsystemConstants.ALGAE_INTAKE_WHEEL_MOTOR_ID, "rio");
    m_velocityDutyCycle = new VelocityDutyCycle(0);
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kP = AlgaeSubsystemConstants.TALON_KP;
    slot0Configs.kI = AlgaeSubsystemConstants.TALON_KI;
    slot0Configs.kD = AlgaeSubsystemConstants.TALON_KD;

    limitConfigs.SupplyCurrentLimit = AlgaeSubsystemConstants.SUPPLY_CURRENT_LIMIT;
    limitConfigs.SupplyCurrentLimitEnable = AlgaeSubsystemConstants.SUPPLY_CURRENT_LIMIT_ENABLE;
    limitConfigs.SupplyCurrentLowerLimit = AlgaeSubsystemConstants.SUPPLY_CURRENT_LOWER_LIMIT;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_holderMotor.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Is algae in", isAlgaeIn());
    SmartDashboard.putBoolean("Is algae out", isAlgaeOut());
    SmartDashboard.putNumber("algae vel", getVelocity());
    SmartDashboard.putNumber("algae voltage", m_holderMotor.getMotorVoltage().getValueAsDouble());
  }

  // public boolean isAlgaeIn() {
  //   return !m_algaeDetector.get();
  // }

  public void setOutput(double output) {
    m_holderMotor.set(output);
  }

  public void setVelocity(double velocity) {
    // m_holderMotor.setControl(m_velocityDutyCycle.withVelocity(velocity));
    m_holderMotor.set(velocity);
  }

  public double getVelocity() {
    return m_holderMotor.getVelocity().getValueAsDouble();
  }

  public double getOutput() {
    return m_holderMotor.get();
  }

  public boolean isAlgaeIn() {
    boolean lastStatus = false;
    if (getOutput() == 0) {
      return lastStatus;
    } else if (getOutput() > 0 && getVelocity() < AlgaeSubsystemConstants.ALGAE_IN_VELOCITY) {
      if (!timer.isRunning()) {
        timer.restart();
      } else if (timer.get() > AlgaeSubsystemConstants.REQ_TIME) {
        return true;
      }
    } else {
      timer.stop();
    }
    return false;
    // if (getOutput() > 0 && getVelocity() < CoralSubsystemConstants.CORAL_IN_VELOCITY) {
    //   if (!startTimer) {
    //     startTimer = true;
    //     checkTimer = true;
    //     timer.start();
    //   }
    // }
    // if (checkTimer) {
    //   if (getTime() > CoralSubsystemConstants.REQ_TIME && getVelocity() < CoralSubsystemConstants.CORAL_IN_VELOCITY) {
    //     startTimer = false;
    //     checkTimer = false;
    //     timer.reset();
    //     return true;
    //   }
    // }
    // return false;
  }

  public boolean isAlgaeOut() {
    boolean lastStatus = false;
    if (getOutput() == 0) {
      return lastStatus;
    } else if (getOutput() < 0 && getVelocity() > AlgaeSubsystemConstants.ALGAE_IN_VELOCITY) {
      if (!timer.isRunning()) {
        timer.restart();
      } else if (timer.get() > AlgaeSubsystemConstants.REQ_TIME) {
        return true;
      }
    } else {
      timer.stop();
    }
    return false;
    // if (getOutput() < 0 && getVelocity() > CoralSubsystemConstants.CORAL_OUT_VELOCITY) {
    //   if (!startTimer) {
    //     startTimer = true;
    //     checkTimer = true;
    //     timer.start();
    //   }
    // }
    // if (checkTimer) {
    //   if (getTime() > CoralSubsystemConstants.REQ_TIME && getVelocity() > CoralSubsystemConstants.CORAL_OUT_VELOCITY) {
    //     startTimer = false;
    //     checkTimer = false;
    //     timer.reset();
    //     return true;
    //   }
    // }
    // return false;
  }

  public double getTime() {
    return timer.get();
  }
}
