package frc.robot;

import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotSubsystems {

  public final ElevatorSubsystem elevator = new ElevatorSubsystem();
  public final ClimbSubsystem climb = new ClimbSubsystem();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final CoralSubsystem coral = new CoralSubsystem();
  public final AlgaeSubsystem algae = new AlgaeSubsystem();
  public final SwerveSubsystem swerve = new SwerveSubsystem();
}
