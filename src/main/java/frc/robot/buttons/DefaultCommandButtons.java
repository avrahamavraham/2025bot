package frc.robot.buttons;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.lib.util.SetSubsystemTargetCommand;
import frc.robot.Enums.CoralPosition;
import frc.robot.Enums.CoralVelocity;
import frc.robot.Enums.ElevatorPosition;
import frc.robot.Enums.IntakePosition;
import frc.robot.RobotControllers;
import frc.robot.RobotState;
import frc.robot.RobotSubsystems;
import frc.robot.commands.swerve.DriveCommand;

public class DefaultCommandButtons {

  private static final int translationAxis = PS5Controller.Axis.kLeftY.value;
  private static final int strafeAxis = PS5Controller.Axis.kLeftX.value;
  private static final int rotationAxis = PS5Controller.Axis.kRightX.value;

  public static void loadButtons(RobotControllers robotControllers, RobotSubsystems robotSubsystems) {
    resetRobotState(robotControllers);
    elevator(robotSubsystems);
    intake(robotSubsystems);
    algae(robotSubsystems);
    coral(robotSubsystems);
    climb(robotSubsystems, robotControllers.subsystem);
    swerve(robotControllers.driver, robotSubsystems);
  }

  public static void resetRobotState(RobotControllers controllers) {
    controllers.subsystem.povDown().onTrue(new InstantCommand(RobotState::resetRobotState));
  }

  public static void elevator(RobotSubsystems robotSubsystems) {
    Runnable setElevatorTargetByState = () -> {
      if (!robotSubsystems.elevator.isFirstResetOccurred()) {
        robotSubsystems.elevator.stop();
      } else if (robotSubsystems.coral.getAngle() < CoralPosition.SafeZone.getTarget()) {
        robotSubsystems.elevator.setLevel(RobotState.elevatorPosition.getTarget());
      }
    };
    Command followElevatorStateTargetCommand = new SetSubsystemTargetCommand(
      robotSubsystems.elevator,
      setElevatorTargetByState
    );
    robotSubsystems.elevator.setDefaultCommand(followElevatorStateTargetCommand);
  }

  public static void intake(RobotSubsystems robotSubsystems) {
    Runnable setIntakeTargetByState = () -> {
      double elevatorDiff = Math.abs(ElevatorPosition.Close.getTarget() - robotSubsystems.elevator.getHeight());
      if (!robotSubsystems.intake.isFirstResetOccurred()) {
        robotSubsystems.intake.stop();
      } else if (
        elevatorDiff < ElevatorPosition.Threshold.getTarget() &&
        robotSubsystems.coral.getAngle() < CoralPosition.SafeZone.getTarget()
      ) {
        robotSubsystems.intake.setAnglePosition(RobotState.intakePosition.getTarget());
        robotSubsystems.intake.setWheelsVelocity(RobotState.intakeVelocity.getTarget());
      }
    };
    Command followIntakeStateTargetCommand = new SetSubsystemTargetCommand(
      robotSubsystems.intake,
      setIntakeTargetByState
    );
    robotSubsystems.intake.setDefaultCommand(followIntakeStateTargetCommand);
  }

  public static void algae(RobotSubsystems robotSubsystems) {
    Runnable setAlgaeTargetByState = () -> {
      double elevatorDiff = Math.abs(RobotState.elevatorPosition.getTarget() - robotSubsystems.elevator.getHeight());
      if (
        elevatorDiff < ElevatorPosition.Threshold.getTarget() &&
        robotSubsystems.coral.getAngle() < CoralPosition.SafeZone.getTarget()
      ) {
        robotSubsystems.algae.setVelocity(RobotState.algaeVelocity.getTarget());
      }
    };
    Command followAlgaeStateTargetCommand = new SetSubsystemTargetCommand(robotSubsystems.algae, setAlgaeTargetByState);
    robotSubsystems.algae.setDefaultCommand(followAlgaeStateTargetCommand);
  }

  public static void coral(RobotSubsystems robotSubsystems) {
    Runnable setCoralTargetByState = () -> {
      double elevatorDiff = Math.abs(RobotState.elevatorPosition.getTarget() - robotSubsystems.elevator.getHeight());
      if (!robotSubsystems.coral.isFirstResetOccurred()) {
        robotSubsystems.coral.stop();
      } else if (elevatorDiff < ElevatorPosition.Threshold.getTarget()) {
        robotSubsystems.coral.setAngle(RobotState.coralPosition.getTarget());
        robotSubsystems.coral.setVelocity(RobotState.coralVelocity.getTarget());
      } else {
        robotSubsystems.coral.setAngle(CoralPosition.Close.getTarget());
        robotSubsystems.coral.setVelocity(CoralVelocity.KeepItIn.getTarget());
      }
    };
    Command followCoralStateTargetCommand = new SetSubsystemTargetCommand(robotSubsystems.coral, setCoralTargetByState);
    robotSubsystems.coral.setDefaultCommand(followCoralStateTargetCommand);
  }

  public static void climb(RobotSubsystems robotSubsystems, CommandPS5Controller subsystemController) {
    Runnable setClimbTargetByState = () -> {
      double elevatorDiff = Math.abs(ElevatorPosition.Close.getTarget() - robotSubsystems.elevator.getHeight());
      double coralDiff = Math.abs(CoralPosition.Close.getTarget() - robotSubsystems.coral.getAngle());
      double intakeDiff = Math.abs(IntakePosition.Close.getTarget() - robotSubsystems.intake.getAngle());
      if (
        elevatorDiff < ElevatorPosition.Threshold.getTarget() &&
        coralDiff < CoralPosition.Threshold.getTarget() &&
        intakeDiff < IntakePosition.ThresHold.getTarget() &&
        subsystemController.L1().getAsBoolean()
      ) {
        robotSubsystems.climb.setOutput(() -> subsystemController.getLeftY());
      }
    };
    Command followClimbStateTargetCommand = new SetSubsystemTargetCommand(robotSubsystems.climb, setClimbTargetByState);
    robotSubsystems.climb.setDefaultCommand(followClimbStateTargetCommand);
  }

  public static void swerve(CommandPS5Controller driverController, RobotSubsystems robotSubsystems) {
    robotSubsystems.swerve.setDefaultCommand(
      new DriveCommand(
        robotSubsystems.swerve,
        () -> -driverController.getRawAxis(translationAxis),
        () -> -driverController.getRawAxis(strafeAxis),
        () -> driverController.getRawAxis(rotationAxis),
        () -> false
      )
    );
  }
}
