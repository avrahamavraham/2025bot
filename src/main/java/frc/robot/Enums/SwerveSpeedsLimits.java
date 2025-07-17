package frc.robot.Enums;

import frc.lib.util.ITarget;

public enum SwerveSpeedsLimits implements ITarget {
  Normal(1),
  halfSpeed(2);

  private double m_speed;

  SwerveSpeedsLimits(double velocity) {
    m_speed = velocity;
  }

  @Override
  public double getTarget() {
    return m_speed;
  }
}
