package frc.robot.Enums;

import frc.lib.util.ITarget;

public enum ElevatorPosition implements ITarget {
  Close(0),
  Human(0.03),
  L1(0),
  L2(0.43),
  L3(0.8),
  L4(1.5),
  SafeZone(0),
  Threshold(0.075),
  Net(2);

  private double m_height;

  private ElevatorPosition(double height) {
    m_height = height;
  }

  @Override
  public double getTarget() {
    return m_height;
  }
}
