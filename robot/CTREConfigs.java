package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.constants.SwerveConstants;

public final class CTREConfigs {

  public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
  public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
  public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

  public CTREConfigs() {
    /** Swerve CANCoder Configuration */
    swerveCANcoderConfig.MagnetSensor.SensorDirection = SwerveConstants.CANCODER_INVERT;

    /** Swerve Angle Motor Configurations */
    /* Motor Inverts and Neutral Mode */
    swerveAngleFXConfig.MotorOutput.Inverted = SwerveConstants.ANGLE_MOTOR_INVERT;
    swerveAngleFXConfig.MotorOutput.NeutralMode = SwerveConstants.ANGLE_NEUTRAL_MODE;

    /* Gear Ratio and Wrapping Config */
    swerveAngleFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.ANGLE_GEAR_RATIO;
    swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

    /* Current Limiting */
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT;
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.ANGLE_CURRENT_LIMIT;
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.ANGLE_CURRENT_THRESHOLD;
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerLimit = SwerveConstants.ANGLE_CURRENT_LOWER_LIMIT;
    swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerTime = SwerveConstants.ANGLE_CURRENT_THRESHOLD_TIME;

    /* PID Config */
    swerveAngleFXConfig.Slot0.kP = SwerveConstants.ANGLE_KP;
    swerveAngleFXConfig.Slot0.kI = SwerveConstants.ANGLE_KI;
    swerveAngleFXConfig.Slot0.kD = SwerveConstants.ANGLE_KD;

    /** Swerve Drive Motor Configuration */
    /* Motor Inverts and Neutral Mode */
    swerveDriveFXConfig.MotorOutput.Inverted = SwerveConstants.DRIVE_MOTOR_INVERT;
    swerveDriveFXConfig.MotorOutput.NeutralMode = SwerveConstants.DRIVE_NEUTRAL_MODE;

    /* Gear Ratio Config */
    swerveDriveFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.DRIVE_GEAR_RATIO;

    /* Current Limiting */
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = SwerveConstants.DRIVE_ENABLE_CURRENT_LIMIT;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.DRIVE_CURRENT_LIMIT;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.DRIVE_CURRENT_THRESHOLD;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = SwerveConstants.DRIVE_CURRENT_LOWER_LIMIT;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = SwerveConstants.DRIVE_CURRENT_THRESHOLD_TIME;

    /* PID Config */
    swerveDriveFXConfig.Slot0.kP = SwerveConstants.DRIVE_KP;
    swerveDriveFXConfig.Slot0.kI = SwerveConstants.DRIVE_KI;
    swerveDriveFXConfig.Slot0.kD = SwerveConstants.DRIVE_KD;

    /* Open and Closed Loop Ramping */
    swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.OPEN_LOOP_RAMP;
    swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.OPEN_LOOP_RAMP;

    swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.CLOSED_LOOP_RAMP;
    swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants.CLOSED_LOOP_RAMP;
  }
}
