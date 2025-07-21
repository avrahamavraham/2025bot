package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import frc.lib.util.CANCandleConstants;
import frc.lib.util.CANCandleConstants.AlgaeInLED;
import frc.lib.util.CANCandleConstants.CoralInLED;
import frc.lib.util.CANCandleConstants.DisableLED;

public class RobotCANdle {

  CANdleConfiguration m_config;
  protected CANdle m_CANdle;

  public RobotCANdle() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = CANCandleConstants.CANDLE_LED_TYPE;
    config.brightnessScalar = CANCandleConstants.CANDLE_BRIGHTNESS_SCALAR;
    m_CANdle.configAllSettings(m_config);
  }

  public void isCoralIn() {
    m_CANdle.setLEDs(CoralInLED.R, CoralInLED.G, CoralInLED.B);
  }

  public void isAlgaeIn() {
    m_CANdle.setLEDs(AlgaeInLED.R, AlgaeInLED.G, AlgaeInLED.B);
  }

  public void disableCANdle() {
    m_CANdle.setLEDs(DisableLED.R, DisableLED.G, DisableLED.B);
  }
}
