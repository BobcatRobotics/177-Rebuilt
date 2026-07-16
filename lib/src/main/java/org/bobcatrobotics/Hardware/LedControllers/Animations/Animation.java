package org.bobcatrobotics.Hardware.LedControllers.Animations;

import java.util.function.BooleanSupplier;
import org.bobcatrobotics.Hardware.LedControllers.LedStrip;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Interface for LED animations that can be controlled through lifecycle methods.
 * 
 * <p>
 * All LED animations implement this interface to provide a consistent API for starting, pausing,
 * stopping, and querying the status of animations. Animations typically manipulate one or more
 * {@link LedStrip}s and are backed by WPILib {@link Command}s when scheduled.
 * </p>
 * 
 * <p>
 * Implementing classes should ensure that {@link #run()} can safely be called repeatedly to
 * continue or restart the animation. {@link #stop()} allows pausing without resetting LED states,
 * and {@link #end()} ensures a clean termination by turning off LEDs.
 * </p>
 */
public interface Animation {

    /**
     * Starts or resumes the animation.
     * 
     * <p>
     * If the animation is not currently scheduled, this schedules it to the command scheduler. If
     * already running, this should have no effect. For countdown or timed animations, calling
     * {@code run()} after completion should optionally restart the animation.
     * </p>
     * 
     * <p>
     * Implementations should update the associated LEDs based on the animation logic.
     * </p>
     */
    void run();

    /**
     * Pauses the animation while maintaining the current LED state.
     * 
     * <p>
     * This cancels any underlying WPILib {@link Command} controlling the animation but does not
     * turn off the LEDs. LEDs remain in their current color or brightness. Call {@link #run()} to
     * resume the animation from its current state.
     * </p>
     */
    void stop();

    /**
     * Stops the animation and turns off all LEDs in the strip.
     * 
     * <p>
     * This cancels any underlying command and sets all LEDs in the animation's associated
     * {@link LedStrip} to off (0, 0, 0, 0). This provides a clean termination of the animation.
     * </p>
     */
    void end();

    /**
     * Checks if the animation is currently active.
     * 
     * <p>
     * Should return {@code true} if the animation is running, and {@code false} otherwise.
     * </p>
     *
     * @return true if the animation is currently running, false otherwise
     */
    public default boolean isRunning() {
        return false;
    }

    /**
     * Creates a WPILib {@link Command} that will run the animation until a specified condition
     * evaluates to true.
     * 
     * <p>
     * This allows animations to be executed in the command scheduler with conditional termination.
     * The command should automatically call {@link #end()} once the condition is met.
     * </p>
     * 
     * @param condition a {@link BooleanSupplier} which, when true, ends the animation
     * @return a {@link Command} wrapping this animation and stopping when the condition is met
     */
    public default Command untill(BooleanSupplier condition) {
        return null;
    }

    /**
     * Creates a WPILib {@link Command} that will run the animation for a specified duration.
     * 
     * <p>
     * Once the timeout elapses, the command should automatically call {@link #end()} to cleanly
     * terminate the animation.
     * </p>
     * 
     * @param seconds the maximum number of seconds to run the animation
     * @return a {@link Command} wrapping this animation with a timeout
     */
    public default Command withTimeout(double seconds) {
        return null;
    }
}
