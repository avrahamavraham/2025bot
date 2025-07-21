package frc.robot.Enums;

import frc.lib.util.ITarget;

public enum CoralPosition implements ITarget {
  Close(0),
  Eject(150),
  L1(109),
  Collect(55),
  SafeZone(24),
  Threshold(1);

  private double m_height;

  private CoralPosition(double height) {
    m_height = height;
  }

  @Override
  public double getTarget() {
    return m_height;
  }
}
