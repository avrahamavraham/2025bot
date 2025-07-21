package frc.robot.Enums;

import frc.lib.util.ITarget;

public enum IntakeVelocity implements ITarget {
  Collect(1),
  Stop(0);

  private double m_velocity;

  IntakeVelocity(double velocity) {
    m_velocity = velocity;
  }

  @Override
  public double getTarget() {
    return m_velocity;
  }
}
