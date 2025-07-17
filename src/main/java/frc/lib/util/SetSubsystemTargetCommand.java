package frc.lib.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SetSubsystemTargetCommand extends Command {

  private Runnable m_callback;

  public SetSubsystemTargetCommand(SubsystemBase subsystem, Runnable callback) {
    addRequirements(subsystem);
    m_callback = callback;
  }

  @Override
  public void execute() {
    m_callback.run();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
