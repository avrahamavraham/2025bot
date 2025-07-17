package frc.lib.util;

import com.ctre.phoenix.led.CANdle.LEDStripType;
import org.w3c.dom.css.RGBColor;

public class CANCandleConstants {

  public static final LEDStripType CANDLE_LED_TYPE = LEDStripType.RGB;
  public static final double CANDLE_BRIGHTNESS_SCALAR = 1;
  public static final int CAN_CANDLE_ID = 0;

  public static class CoralInLED {

    public static final int R = 128;
    public static final int G = 0;
    public static final int B = 128;
  }

  public static class AlgaeInLED {

    public static final int R = 255;
    public static final int G = 192;
    public static final int B = 203;
  }

  public static class DisableLED {

    public static final int R = 128;
    public static final int G = 128;
    public static final int B = 128;
  }
}
