package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.buttons.AlgaeButtons;
import frc.robot.buttons.CoralButtons;
import frc.robot.buttons.DefaultCommandButtons;
import frc.robot.buttons.SwerveButtons;

public class RobotContainer {

  private final RobotSubsystems robotSubsystems = new RobotSubsystems();
  private final RobotControllers robotControllers = new RobotControllers();

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    DefaultCommandButtons.loadButtons(robotControllers, robotSubsystems);
    AlgaeButtons.loadButtons(robotControllers, robotSubsystems);
    CoralButtons.loadButtons(robotControllers, robotSubsystems);
    SwerveButtons.loadButtons(robotControllers, robotSubsystems);
    // ClimbButtons.loadButtons(robotControllers, robotSubsystems);
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("example auto");
  }
}
