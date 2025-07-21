package frc.robot.Enums;

import frc.lib.util.ITarget;

public enum ClimbPosition implements ITarget {
  Open(0),
  Close(0),
  Hold(0);

  private double m_angle;

  private ClimbPosition(double angle) {
    m_angle = angle;
  }

  @Override
  public double getTarget() {
    return m_angle;
  }
}
