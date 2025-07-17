package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Enums.AlgaeVelocity;
import frc.robot.Enums.CoralPosition;
import frc.robot.Enums.CoralVelocity;
import frc.robot.Enums.ElevatorPosition;
import frc.robot.Enums.IntakePosition;
import frc.robot.Enums.IntakeVelocity;
import frc.robot.RobotControllers;
import frc.robot.RobotState;
import frc.robot.RobotSubsystems;

public class AlgaeButtons {

  public static void loadButtons(RobotControllers robotControllers, RobotSubsystems robotSubsystems) {
    collect(robotControllers.subsystem, robotSubsystems);
    ground(robotControllers.subsystem, robotSubsystems);
    high(robotControllers.subsystem, robotSubsystems);
    low(robotControllers.subsystem, robotSubsystems);
    net(robotControllers.subsystem, robotSubsystems);
    eject(robotControllers, robotSubsystems);
  }

  protected static void collect(CommandPS5Controller subsystemController, RobotSubsystems robotSubsystems) {
    Trigger collectAlgaeTrigger = subsystemController.povLeft().and(subsystemController.L2());
    collectAlgaeTrigger.onTrue(
      new InstantCommand(() -> {
        RobotState.algaeVelocity = AlgaeVelocity.Collect;
        RobotState.intakePosition = IntakePosition.Collect;
        RobotState.intakeVelocity = IntakeVelocity.Collect;
        RobotState.coralVelocity = CoralVelocity.KeepItIn;
        RobotState.coralPosition = CoralPosition.Close;
      })
    );
    collectAlgaeTrigger.onFalse(
      new InstantCommand(() -> {
        RobotState.algaeVelocity = AlgaeVelocity.KeepItIn;
        RobotState.intakePosition = IntakePosition.Close;
        RobotState.intakeVelocity = IntakeVelocity.Stop;
      })
    );
  }

  protected static void ground(CommandPS5Controller subsystemController, RobotSubsystems robotSubsystems) {
    Trigger algaeGroundTrigger = subsystemController.povLeft().and(subsystemController.cross());
    algaeGroundTrigger.onTrue(
      new InstantCommand(() -> {
        RobotState.elevatorPosition = ElevatorPosition.Close;
      })
    );
  }

  public static void low(CommandPS5Controller subsystemController, RobotSubsystems robotSubsystems) {
    Trigger algaeLowReef = subsystemController.povLeft().and(subsystemController.circle());
    algaeLowReef.onTrue(
      new InstantCommand(() -> {
        RobotState.elevatorPosition = ElevatorPosition.L2;
      })
    );
  }

  public static void high(CommandPS5Controller subsystemController, RobotSubsystems robotSubsystems) {
    Trigger algaeHighReef = subsystemController.povLeft().and(subsystemController.square());
    algaeHighReef.onTrue(
      new InstantCommand(() -> {
        RobotState.elevatorPosition = ElevatorPosition.L3;
      })
    );
  }

  protected static void net(CommandPS5Controller subsystemController, RobotSubsystems robotSubsystems) {
    Trigger algaeNet = subsystemController.povLeft().and(subsystemController.triangle());
    algaeNet.onTrue(
      new InstantCommand(() -> {
        RobotState.elevatorPosition = ElevatorPosition.Net;
      })
    );
  }

  public static void eject(RobotControllers robotControllers, RobotSubsystems robotSubsystems) {
    Trigger algaeEjectSubsystemsController = robotControllers.subsystem.povLeft().and(robotControllers.subsystem.R2());
    Trigger algaeEjectDriverController = robotControllers.driver.povLeft().and(robotControllers.driver.R2());

    InstantCommand startEjectCommand = new InstantCommand(() -> {
      RobotState.algaeVelocity = AlgaeVelocity.Eject;
    });
    InstantCommand endEjectCommand = new InstantCommand(() -> {
      RobotState.algaeVelocity = AlgaeVelocity.KeepItIn;
    });

    algaeEjectSubsystemsController.onTrue(startEjectCommand);
    algaeEjectSubsystemsController.onFalse(endEjectCommand);

    algaeEjectDriverController.onTrue(startEjectCommand);
    algaeEjectDriverController.onFalse(endEjectCommand);
  }
}
