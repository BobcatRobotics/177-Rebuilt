package org.bobcatrobotics.Hardware.LedControllers.Animations;

import java.util.function.BooleanSupplier;
import org.bobcatrobotics.Hardware.LedControllers.LedControllerIO;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Implements a "blink" LED animation for a specified LED strip.
 * <p>
 * The animation sets the LEDs to a specified color and maintains it for a defined period,
 * then marks the animation as finished. Supports running, stopping (pausing), ending (turning off),
 * and timed or conditional execution.
 * </p>
 */
public class Blink implements Animation {

    /** Indicates if the animation has finished. */
    public boolean isFinished = false;

    /** The LED controller being animated. */
    private LedControllerIO ledController;

    /** The start time of the animation in milliseconds. */
    private long startTimeMs = 0;

    /** The color to blink the LEDs. */
    private Color color;

    /** The duration to maintain the color in milliseconds. */
    private long period;

    /**
     * Constructs a new Blink animation for a given LED controller, color, and duration.
     *
     * @param ledController the LED controller to animate
     * @param color         the color to blink
     * @param period        the duration in milliseconds to keep the LEDs on
     */
    public Blink(LedControllerIO ledController, Color color, long period) {
        this.ledController = ledController;
        this.color = color;
        this.startTimeMs = System.currentTimeMillis();
        this.period = period;
    }

    /**
     * Starts or resumes the animation.
     * <p>
     * If the animation has already finished, this does nothing. Otherwise, it sets the LEDs
     * to the specified color and checks if the period has elapsed to mark the animation as finished.
     * </p>
     */
    public void run() {
        if (isFinished) {
            return;
        }
        long currentTimeMs = System.currentTimeMillis();
        ledController.setLEDs(
                (int) (color.red),
                (int) (color.green),
                (int) (color.blue),
                0,
                ledController.getStrip().startIndex,
                ledController.getStrip().endIndex
        );
        if ((currentTimeMs - startTimeMs) >= period) {
            isFinished = true;
        }
    }

    /**
     * Pauses the animation while maintaining the current LED state.
     * <p>
     * LEDs remain in their current color. Call {@link #run()} to resume.
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
     * Marks the animation as finished and sets all LEDs in the affected segment to off (0, 0, 0, 0).
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
