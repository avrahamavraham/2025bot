package frc.robot.Enums;

import frc.lib.util.ITarget;

public enum AlgaeVelocity implements ITarget {
  KeepItIn(0),
  Collect(1),
  Eject(-1);

  private double m_velocity;

  private AlgaeVelocity(double velocity) {
    m_velocity = velocity;
  }

  @Override
  public double getTarget() {
    return m_velocity;
  }
}
