package org.bobcatrobotics.Hardware.LedControllers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for controlling LED strips via a {@link LedControllerIO} implementation.
 * <p>
 * Provides methods to set LED colors, run animations, turn off LEDs, and log
 * input states for monitoring purposes.
 * </p>
 */
public class LedController extends SubsystemBase {
    private final LedControllerIO io;
    private final LedControllerIOInputsAutoLogged inputs =
            new LedControllerIOInputsAutoLogged();

    /**
     * Constructs a new {@link LedController} with the specified IO implementation.
     *
     * @param io the {@link LedControllerIO} implementation used to control the LEDs
     */
    public LedController(LedControllerIO io) {
        this.io = io;
    }

    /**
     * Periodic method called once per scheduler run.
     * <p>
     * Updates inputs from the hardware and logs them via the {@link Logger}.
     * </p>
     */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Led Controller", inputs);
    }

    /**
     * Sets the LEDs to a solid color for a specified range.
     *
     * @param r        red component (0-255)
     * @param g        green component (0-255)
     * @param b        blue component (0-255)
     * @param w        white component (0-255)
     * @param startIdx starting LED index (inclusive)
     * @param endIdx   ending LED index (exclusive)
     */
    public void setLEDs(int r, int g, int b, int w, int startIdx, int endIdx) {
        inputs.animation = "Solid";
        inputs.running = true;
        inputs.brightnessScale = 1.0;
        io.setLEDs(r, g, b, w, startIdx, endIdx - startIdx);
    }

    /**
     * Placeholder method for setting an LED animation.
     * <p>
     * Implement animation logic as needed.
     * </p>
     */
    public void setAnimation() {
        // TODO: Implement animation logic
    }

    /**
     * Turns off the LEDs for a specified range.
     *
     * @param startIdx starting LED index (inclusive)
     * @param endIdx   ending LED index (exclusive)
     */
    public void off(int startIdx, int endIdx) {
        inputs.animation = "Off";
        inputs.running = false;
        io.setLEDs(0, 0, 0, 0, startIdx, endIdx - startIdx);
    }

    /**
     * Returns the total number of LEDs in the strip.
     *
     * @return the LED count
     */
    public int getCount() {
        return io.getCount();
    }
}
