package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Enums.CoralPosition;
import frc.robot.Enums.CoralVelocity;
import frc.robot.Enums.ElevatorPosition;
import frc.robot.RobotControllers;
import frc.robot.RobotState;
import frc.robot.RobotSubsystems;

public class CoralButtons {

  public static void loadButtons(RobotControllers robotControllers, RobotSubsystems robotSubsystems) {
    collect(robotControllers.subsystem, robotSubsystems);
    l1(robotControllers.subsystem, robotSubsystems);
    l2(robotControllers.subsystem, robotSubsystems);
    l3(robotControllers.subsystem, robotSubsystems);
    l4(robotControllers.subsystem, robotSubsystems);
    eject(robotControllers, robotSubsystems);
  }

  public static void collect(CommandPS5Controller subsystemController, RobotSubsystems robotSubsystems) {
    Trigger collectCoralTrigger = subsystemController.povRight().and(subsystemController.L2());
    collectCoralTrigger.onTrue(
      new InstantCommand(() -> {
        RobotState.coralVelocity = CoralVelocity.Collect;
        RobotState.coralPosition = CoralPosition.Collect;
        RobotState.elevatorPosition = ElevatorPosition.Human;
      })
    );
    collectCoralTrigger.onFalse(
      new InstantCommand(() -> {
        RobotState.coralVelocity = CoralVelocity.KeepItIn;
        RobotState.coralPosition = CoralPosition.Close;
        RobotState.elevatorPosition = ElevatorPosition.Close;
      })
    );
  }

  public static void l1(CommandPS5Controller subsystemController, RobotSubsystems robotSubsystems) {
    Trigger coralLevel1 = subsystemController.povRight().and(subsystemController.cross());
    coralLevel1.onTrue(
      new InstantCommand(() -> {
        RobotState.coralVelocity = CoralVelocity.KeepItIn;
        RobotState.coralPosition = CoralPosition.L1;
        RobotState.elevatorPosition = ElevatorPosition.L1;
      })
    );
  }

  public static void l2(CommandPS5Controller subsystemController, RobotSubsystems robotSubsystems) {
    Trigger coralLevel2 = subsystemController.povRight().and(subsystemController.circle());
    coralLevel2.onTrue(
      new InstantCommand(() -> {
        RobotState.coralVelocity = CoralVelocity.KeepItIn;
        RobotState.coralPosition = CoralPosition.Eject;
        RobotState.elevatorPosition = ElevatorPosition.L2;
      })
    );
  }

  public static void l3(CommandPS5Controller subsystemController, RobotSubsystems robotSubsystems) {
    Trigger coralLevel3 = subsystemController.povRight().and(subsystemController.square());
    coralLevel3.onTrue(
      new InstantCommand(() -> {
        RobotState.coralVelocity = CoralVelocity.KeepItIn;
        RobotState.coralPosition = CoralPosition.Eject;
        RobotState.elevatorPosition = ElevatorPosition.L3;
      })
    );
  }

  public static void l4(CommandPS5Controller subsystemController, RobotSubsystems robotSubsystems) {
    Trigger coralLevel4 = subsystemController.povRight().and(subsystemController.triangle());
    coralLevel4.onTrue(
      new InstantCommand(() -> {
        RobotState.coralVelocity = CoralVelocity.KeepItIn;
        RobotState.coralPosition = CoralPosition.Eject;
        RobotState.elevatorPosition = ElevatorPosition.L4;
      })
    );
  }

  public static void eject(RobotControllers robotControllers, RobotSubsystems robotSubsystems) {
    Trigger ejectCoralSubsystemController = robotControllers.subsystem.povRight().and(robotControllers.subsystem.R2());
    Trigger ejectCoralDriverController = robotControllers.driver.povRight().and(robotControllers.driver.R2());

    InstantCommand startEjectCoral = new InstantCommand(() -> {
      RobotState.coralVelocity = CoralVelocity.Eject;
    });
    InstantCommand endEjectCoral = new InstantCommand(() -> {
      RobotState.coralVelocity = CoralVelocity.KeepItIn;
    });

    ejectCoralSubsystemController.onTrue(startEjectCoral);
    ejectCoralSubsystemController.onFalse(endEjectCoral);

    ejectCoralDriverController.onTrue(startEjectCoral);
    ejectCoralDriverController.onFalse(endEjectCoral);
  }
}
