package org.bobcatrobotics.Hardware.LedControllers;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

/**
 * Simulation implementation of a LED controller.
 * <p>
 * This class implements {@link LedControllerIO} and simulates controlling an LED strip.
 * It does not interact with real hardware, but mimics behavior for testing or simulation purposes.
 * </p>
 */
public class LedSim implements LedControllerIO {

    /** The simulated LED strip */
    public LedStrip strip;

    /** Last red value set */
    private int r;

    /** Last green value set */
    private int g;

    /** Last blue value set */
    private int b;

    /** Last white value set */
    private int w;

    /** Starting LED index for the last set operation */
    private int start;

    /** Ending LED index for the last set operation */
    private int end;

    /**
     * Constructs a simulated LED controller for a given strip.
     *
     * @param canId     simulated CAN ID (not used in simulation)
     * @param strip     the {@link LedStrip} instance to simulate
     * @param stripType the type of LED strip (RGB, RGBW, etc.)
     */
    public LedSim(int canId, LedStrip strip, StripTypeValue stripType) {
        this.strip = strip;
    }

    /**
     * Sets the simulated LEDs to a specified color over a range.
     *
     * @param r        red component (0-255)
     * @param g        green component (0-255)
     * @param b        blue component (0-255)
     * @param w        white component (0-255)
     * @param startIdx starting LED index (inclusive)
     * @param endIdx   ending LED index (exclusive)
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
    }

    /**
     * Updates the {@link LedControllerIOInputs} object with the last set LED values.
     *
     * @param inputs the inputs object to populate
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
     * Returns the simulated LED strip.
     *
     * @return the {@link LedStrip} instance
     */
    public LedStrip getStrip() {
        return strip;
    }

    /**
     * Returns the total number of LEDs in the simulated strip.
     *
     * @return the number of LEDs
     */
    public int getCount() {
        return strip.length;
    }
}
