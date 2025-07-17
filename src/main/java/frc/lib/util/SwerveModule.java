package frc.lib.util;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.robot.Robot;
import frc.robot.constants.SwerveConstants;

public class SwerveModule {

  public int moduleNumber;
  private Rotation2d angleOffset;

  private TalonFX m_angleMotor;
  private TalonFX m_driveMotor;
  private CANcoder m_angleEncoder;

  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(
    SwerveConstants.DRIVE_KP,
    SwerveConstants.DRIVE_KV,
    SwerveConstants.DRIVE_KA
  );

  /* drive motor control requests */
  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

  /* angle motor control requests */
  private final PositionVoltage anglePosition = new PositionVoltage(0);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    m_angleEncoder = new CANcoder(moduleConstants.cancoderID, "canv");
    m_angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

    /* Angle Motor Config */
    m_angleMotor = new TalonFX(moduleConstants.angleMotorID, "canv");
    m_angleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
    resetToAbsolute();

    /* Drive Motor Config */
    m_driveMotor = new TalonFX(moduleConstants.driveMotorID, "canv");
    m_driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
    m_driveMotor.getConfigurator().setPosition(0.0);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    SmartDashboard.putString("mod " + moduleNumber + " Desired Pos", desiredState.toString());

    m_angleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
    setSpeed(desiredState, isOpenLoop);
  }

  public void forceSetAngle(Rotation2d angle) {
    m_angleMotor.setControl(anglePosition.withPosition(angle.getRotations()));
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      driveDutyCycle.Output = desiredState.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
      m_driveMotor.setControl(driveDutyCycle);
    } else {
      driveVelocity.Velocity = Conversions.MPSToRPS(
        desiredState.speedMetersPerSecond,
        SwerveConstants.WHEEL_CIRCUMFERENCE
      );
      driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
      m_driveMotor.setControl(driveVelocity);
    }
  }

  public Rotation2d getCANcoder() {
    return Rotation2d.fromRotations(m_angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public void resetToAbsolute() {
    double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
    m_angleMotor.setPosition(absolutePosition);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      Conversions.RPSToMPS(m_driveMotor.getVelocity().getValueAsDouble(), SwerveConstants.WHEEL_CIRCUMFERENCE),
      Rotation2d.fromRotations(m_angleMotor.getPosition().getValueAsDouble())
    );
  }

  public void resetDriveEncoder() {
    m_driveMotor.setPosition(0);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      Conversions.rotationsToMeters(m_driveMotor.getPosition().getValueAsDouble(), SwerveConstants.WHEEL_CIRCUMFERENCE),
      Rotation2d.fromRotations(m_angleMotor.getPosition().getValueAsDouble())
    );
  }

  public TalonFX getDriveMotor() {
    return m_driveMotor;
  }
}
