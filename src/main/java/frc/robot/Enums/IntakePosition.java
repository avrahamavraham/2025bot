package frc.robot.Enums;

import frc.lib.util.ITarget;

public enum IntakePosition implements ITarget {
  Close(0),
  Eject(-160),
  ThresHold(1),
  Collect(-50);

  private double m_height;

  private IntakePosition(double height) {
    m_height = height;
  }

  @Override
  public double getTarget() {
    return m_height;
  }
}
