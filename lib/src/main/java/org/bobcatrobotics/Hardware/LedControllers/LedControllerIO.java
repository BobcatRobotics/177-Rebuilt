package org.bobcatrobotics.Hardware.LedControllers;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for LED controller hardware abstraction.
 * <p>
 * Provides methods to set LED colors, update input states, and retrieve LED strip details.
 * Implementations can control real hardware or provide a simulation version for testing.
 * </p>
 */
public interface LedControllerIO {

    /**
     * Data class representing the current state of the LED controller.
     * <p>
     * Annotated with {@link AutoLog} to automatically log values in AdvantageKit.
     * </p>
     */
    @AutoLog
    public static class LedControllerIOInputs {
        /** Red color value (0-255) */
        public int red = 0;

        /** Green color value (0-255) */
        public int green = 0;

        /** Blue color value (0-255) */
        public int blue = 0;

        /** White color value (0-255) */
        public int white = 0;

        /** Starting LED index (inclusive) for the current operation */
        public int startIdx = 0;

        /** Ending LED index (exclusive) for the current operation */
        public int endIdx = 0;

        /** Total number of LEDs affected or present on the strip */
        public int count = 0;

        /** Whether the LEDs are currently active/running */
        public boolean running = false;

        /** Name of the current animation or effect */
        public String animation = "None";

        /** Brightness scale factor (0.0-1.0) applied to the LEDs */
        public double brightnessScale = 1.0;
    }

    /**
     * Sets the LEDs to a specified color over a range.
     *
     * @param r        red component (0-255)
     * @param g        green component (0-255)
     * @param b        blue component (0-255)
     * @param w        white component (0-255)
     * @param startIdx starting LED index (inclusive)
     * @param endIdx   ending LED index (exclusive)
     */
    public default void setLEDs(int r, int g, int b, int w, int startIdx, int endIdx) {}

    /**
     * Updates the input state object with the current LED status.
     *
     * @param inputs the {@link LedControllerIOInputs} object to populate
     */
    public default void updateInputs(LedControllerIOInputs inputs) {}

    /**
     * Returns the {@link LedStrip} object representing the physical or virtual LED strip.
     *
     * @return the {@link LedStrip} instance, or {@code null} if unavailable
     */
    public default LedStrip getStrip() {
        return null;
    }

    /**
     * Returns the total number of LEDs on the strip.
     *
     * @return the number of LEDs
     */
    public default int getCount() {
        return 0;
    }
}
