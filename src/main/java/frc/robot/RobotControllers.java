package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.constants.OperatorConstants;

public class RobotControllers {

  public final CommandPS5Controller driver = new CommandPS5Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

  public final CommandPS5Controller subsystem = new CommandPS5Controller(OperatorConstants.SUBSYSTEM_CONTROLLER_PORT);
}
