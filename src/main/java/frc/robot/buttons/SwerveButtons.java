package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotControllers;
import frc.robot.RobotSubsystems;

public class SwerveButtons {

  public static void loadButtons(RobotControllers robotControllers, RobotSubsystems robotSubsystems) {
    robotControllers.driver.touchpad().onTrue(new InstantCommand(() -> robotSubsystems.swerve.resetGyro()));
    robotControllers.driver
      .R1()
      .onTrue(Commands.runOnce(() -> robotSubsystems.swerve.resetPose(), robotSubsystems.swerve));
    robotControllers.driver
      .R2()
      .onTrue(Commands.runOnce(() -> robotSubsystems.swerve.resetDriveEncoders(), robotSubsystems.swerve));
    robotControllers.driver
      .circle()
      .whileTrue(Commands.runOnce(() -> robotSubsystems.swerve.moveFrontLeft(), robotSubsystems.swerve));
  }
}
