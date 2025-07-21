package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Enums.AlgaeVelocity;
import frc.robot.Enums.ClimbPosition;
import frc.robot.Enums.CoralPosition;
import frc.robot.Enums.CoralVelocity;
import frc.robot.Enums.ElevatorPosition;
import frc.robot.Enums.IntakePosition;
import frc.robot.Enums.IntakeVelocity;

public class RobotState {

  public static CoralPosition coralPosition = CoralPosition.Close;
  public static CoralVelocity coralVelocity = CoralVelocity.KeepItIn;
  public static AlgaeVelocity algaeVelocity = AlgaeVelocity.KeepItIn;
  public static ElevatorPosition elevatorPosition = ElevatorPosition.Close;
  public static IntakeVelocity intakeVelocity = IntakeVelocity.Stop;
  public static IntakePosition intakePosition = IntakePosition.Close;
  public static ClimbPosition climbPosition = ClimbPosition.Hold;

  public static void resetRobotState() {
    RobotState.coralPosition = CoralPosition.Close;
    RobotState.coralVelocity = CoralVelocity.KeepItIn;
    RobotState.algaeVelocity = AlgaeVelocity.KeepItIn;
    RobotState.elevatorPosition = ElevatorPosition.Close;
    RobotState.intakeVelocity = IntakeVelocity.Stop;
    RobotState.intakePosition = IntakePosition.Close;
    RobotState.climbPosition = ClimbPosition.Hold;
  }

  public static void sendRobotStateToDashboard() {
    SmartDashboard.putString("RobotState - CoralPosition", RobotState.coralPosition.name());
    SmartDashboard.putString("RobotState - CoralVelocity", RobotState.coralVelocity.name());
    SmartDashboard.putString("RobotState - AlgaeVelocity", RobotState.algaeVelocity.name());
    SmartDashboard.putString("RobotState - ElevatorPosition", RobotState.elevatorPosition.name());
    SmartDashboard.putString("RobotState - IntakeVelocity", RobotState.intakeVelocity.name());
    SmartDashboard.putString("RobotState - IntakePosition", RobotState.intakePosition.name());
    SmartDashboard.putString("RobotState - ClimbPosition", RobotState.climbPosition.name());
  }
}
