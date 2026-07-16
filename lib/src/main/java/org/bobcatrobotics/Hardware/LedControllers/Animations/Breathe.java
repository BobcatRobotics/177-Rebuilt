package org.bobcatrobotics.Hardware.LedControllers.Animations;

import java.util.function.BooleanSupplier;
import org.bobcatrobotics.Hardware.LedControllers.LedControllerIO;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Implements a "breathing" LED animation for a specified LED strip.
 * <p>
 * The animation smoothly fades the LEDs in and out with a configurable frequency, dimness,
 * phase shift, and color. Supports running, stopping (pausing), ending (turning off), and
 * timed or conditional execution.
 * </p>
 */
public class Breathe implements Animation {

    /** Indicates if the animation has finished. */
    public boolean isFinished = false;

    /** The LED controller being animated. */
    private LedControllerIO ledController;

    /** Frequency of the breathing effect in Hz. */
    private double frequency;

    /** Minimum brightness scale (0.0 = off, 1.0 = full brightness). */
    private double dimmness;

    /** Phase shift applied to the breathing sine wave. */
    private double phaseShift;

    /** Color of the LEDs during the animation. */
    private Color color;

    /**
     * Constructs a new Breathe animation for a given LED controller and color.
     *
     * @param ledController the LED controller to animate
     * @param color         the base color of the breathing animation
     */
    public Breathe(LedControllerIO ledController, Color color) {
        this.ledController = ledController;
        this.color = color;
    }

    /**
     * Starts or resumes the animation.
     * <p>
     * If the animation is already finished, this does nothing. Otherwise, it calculates
     * the current brightness based on a sine wave and updates the LED strip.
     * </p>
     */
    public void run() {
        if (isFinished) {
            return;
        }
        long currentTimeMs = System.currentTimeMillis();
        double periodMs = 1000.0 / frequency;
        double timeInPeriod = currentTimeMs % periodMs;
        double phase = (timeInPeriod / periodMs) * 2 * Math.PI + phaseShift;
        double brightness = (Math.sin(phase) + 1) / 2;
        double scale = dimmness + (brightness * (1.0 - dimmness));
        ledController.setLEDs(
                (int) (color.red * scale),
                (int) (color.green * scale),
                (int) (color.blue * scale),
                0,
                ledController.getStrip().startIndex,
                ledController.getStrip().endIndex
        );
    }

    /**
     * Pauses the animation while maintaining the current LED state.
     * <p>
     * The LEDs remain at their current brightness and color. Call {@link #run()} to resume.
     * </p>
     */
    public void stop() {
        ledController.setLEDs(
                (int) (color.red),
                (int) (color.green),
                (int) (color.blue),
                0,
                ledController.getStrip().startIndex,
                ledController.getStrip().endIndex
        );
    }

    /**
     * Ends the animation and turns off all LEDs in the strip segment.
     * <p>
     * Marks the animation as finished and sets all LEDs in the affected segment to off
     * (0, 0, 0, 0).
     * </p>
     */
    public void end() {
        ledController.setLEDs(
                0, 0, 0, 0,
                ledController.getStrip().startIndex,
                ledController.getStrip().endIndex
        );
        isFinished = true;
    }

    /**
     * Checks if the animation is currently active.
     *
     * @return true if the animation is running, false if it has finished
     */
    public boolean isRunning() {
        return isFinished;
    }

    /**
     * Creates a command that runs the animation until a given condition is met.
     *
     * @param condition the condition to stop the animation
     * @return a {@link Command} that will run the animation until the condition is true
     */
    public Command untill(BooleanSupplier condition) {
        return Commands.run(() -> run())
                .unless(condition)
                .andThen(() -> end());
    }

    /**
     * Creates a command that runs the animation for a specified duration.
     *
     * @param seconds the maximum duration to run the animation
     * @return a {@link Command} that will run the animation with a timeout
     */
    public Command withTimeout(double seconds) {
        return Commands.run(() -> run())
                .withTimeout(seconds)
                .andThen(() -> end());
    }
}
