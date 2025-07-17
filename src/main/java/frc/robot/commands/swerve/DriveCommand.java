package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {

  private SwerveSubsystem s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;

  public DriveCommand(
    SwerveSubsystem s_Swerve,
    DoubleSupplier translationSup,
    DoubleSupplier strafeSup,
    DoubleSupplier rotationSup,
    BooleanSupplier robotCentricSup
  ) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal =
      MathUtil.applyDeadband(translationSup.getAsDouble(), OperatorConstants.STICK_DEADBAND) /
      SwerveConstants.SLOW_DRIVE;
    double strafeVal =
      MathUtil.applyDeadband(strafeSup.getAsDouble(), OperatorConstants.STICK_DEADBAND) / SwerveConstants.SLOW_DRIVE;
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), OperatorConstants.STICK_DEADBAND);

    /* Drive */
    s_Swerve.drive(
      new Translation2d(translationVal, strafeVal).times(SwerveConstants.MAX_SPEED),
      rotationVal * SwerveConstants.MAX_ANGULAR_VELOCITY,
      !robotCentricSup.getAsBoolean(),
      true
    );
  }
}
