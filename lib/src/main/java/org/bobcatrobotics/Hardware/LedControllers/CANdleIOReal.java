package org.bobcatrobotics.Hardware.LedControllers;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

/**
 * Implementation of {@link LedControllerIO} for real CANdle hardware.
 * <p>
 * Provides control over an LED strip connected to a CTRE CANdle controller.
 * Supports setting RGBW values for a specified range of LEDs and updating
 * input states for logging or feedback purposes.
 * </p>
 */
public class CANdleIOReal implements LedControllerIO {
    private final CANdle candle;
    public LedStrip strip;

    private int r, g, b, w, start, end;

    /**
     * Constructs a new CANdleIOReal instance.
     *
     * @param canId     the CAN ID of the CANdle device
     * @param strip     the LED strip object associated with this controller
     * @param stripType the type of LED strip connected (e.g., RGB, RGBW)
     */
    public CANdleIOReal(int canId, LedStrip strip, StripTypeValue stripType) {
        candle = new CANdle(canId);
        this.strip = strip;

        CANdleConfiguration cfg = new CANdleConfiguration();
        cfg.LED.StripType = stripType;
        candle.getConfigurator().apply(cfg);
    }

    /**
     * Sets the LED colors for a specified range of the strip.
     *
     * @param r        red component (0-255)
     * @param g        green component (0-255)
     * @param b        blue component (0-255)
     * @param w        white component (0-255)
     * @param startIdx start index of the LED range (inclusive)
     * @param endIdx   end index of the LED range (exclusive)
     */
    public void setLEDs(int r, int g, int b, int w, int startIdx, int endIdx) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.w = w;
        this.start = startIdx;
        this.end = endIdx;
        SolidColor color = new SolidColor(startIdx, endIdx - startIdx);
        RGBWColor rgbwColor = new RGBWColor(r, g, b, w);
        color.withColor(rgbwColor);
        candle.setControl(color);
    }

    /**
     * Updates the given {@link LedControllerIOInputs} object with the
     * current LED state for logging or monitoring.
     *
     * @param inputs the {@link LedControllerIOInputs} object to update
     */
    public void updateInputs(LedControllerIOInputs inputs) {
        inputs.red = r;
        inputs.green = g;
        inputs.blue = b;
        inputs.white = w;
        inputs.startIdx = start;
        inputs.endIdx = end;
        inputs.count = end - start;
    }

    /**
     * Returns the LED strip associated with this controller.
     *
     * @return the {@link LedStrip} object
     */
    public LedStrip getStrip() {
        return strip;
    }

    /**
     * Returns the total number of LEDs in the strip.
     *
     * @return the length of the LED strip
     */
    public int getCount() {
        return strip.length;
    }
}
