package org.bobcatrobotics.Controllers.Gamepads;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Wrapper class for a generic HID (Human Interface Device) controller.
 * <p>
 * Provides named access to buttons, bumpers, triggers, axes, and POV directions.
 * Can be used with WPILib {@link Trigger}-based command bindings.
 * </p>
 */
public class GenericHIDController implements ControllerBase {

    /** Underlying WPILib GenericHID controller. */
    private final GenericHID controller;

    /** Alert shown when the controller is disconnected. */
    private final Alert controllerUnpluggedAlert;

    /** Mapping of human-readable button names to hardware button indices. */
    private Map<String, Integer> buttonMap = new HashMap<>();

    /**
     * Constructs a new GenericHIDController.
     *
     * @param port the USB port of the controller
     * @param name a human-readable name used for alerts
     */
    public GenericHIDController(int port, String name) {
        controllerUnpluggedAlert =
                new Alert(name + " Joystick unplugged!", AlertType.kWarning);
        controller = new GenericHID(port);

        buttonMap = Map.ofEntries(
                Map.entry("A", 1),
                Map.entry("B", 2),
                Map.entry("X", 3),
                Map.entry("Y", 4),
                Map.entry("Select", 9),
                Map.entry("Start", 10)
        );
    }

    /**
     * Returns a trigger for a named button.
     *
     * @param buttonName the human-readable button name
     * @return a {@link Trigger} that is active when the button is pressed
     */
    public Trigger getButton(String buttonName) {
        return new Trigger(() -> controller.getRawButton(buttonMap.get(buttonName).intValue()));
    }

    /**
     * Returns a trigger representing the left trigger button.
     * <p>
     * Currently implemented as a placeholder (always reads button 0).
     * </p>
     *
     * @return a {@link Trigger} for the left trigger
     */
    public Trigger getLeftTrigger() {
        return new Trigger(() -> controller.getRawButton(0));
    }

    /**
     * Returns a trigger representing the right trigger button.
     * <p>
     * Currently implemented as a placeholder (always reads button 0).
     * </p>
     *
     * @return a {@link Trigger} for the right trigger
     */
    public Trigger getRightTrigger() {
        return new Trigger(() -> controller.getRawButton(0));
    }

    /**
     * Returns a trigger representing the left bumper.
     * <p>
     * Currently implemented as a placeholder (always reads button 0).
     * </p>
     *
     * @return a {@link Trigger} for the left bumper
     */
    public Trigger getLeftBumper() {
        return new Trigger(() -> controller.getRawButton(0));
    }

    /**
     * Returns a trigger representing the right bumper.
     * <p>
     * Currently implemented as a placeholder (always reads button 0).
     * </p>
     *
     * @return a {@link Trigger} for the right bumper
     */
    public Trigger getRightBumper() {
        return new Trigger(() -> controller.getRawButton(0));
    }

    /**
     * Returns the X-axis value of the left stick.
     *
     * @return left stick X-axis value
     */
    public double getLeftX() {
        return controller.getRawAxis(0);
    }

    /**
     * Returns the X-axis value of the right stick.
     *
     * @return right stick X-axis value
     */
    public double getRightX() {
        return controller.getRawAxis(0);
    }

    /**
     * Returns the Y-axis value of the left stick.
     *
     * @return left stick Y-axis value
     */
    public double getLeftY() {
        return controller.getRawAxis(0);
    }

    /**
     * Returns the Y-axis value of the right stick.
     *
     * @return right stick Y-axis value
     */
    public double getRightY() {
        return controller.getRawAxis(0);
    }

    /**
     * Updates the controller connection alert.
     * <p>
     * Should be called periodically to reflect the current connection state.
     * </p>
     */
    public void updateControllerAlerts() {
        controllerUnpluggedAlert.set(!controller.isConnected());
    }

    /**
     * Returns a trigger for the POV Up direction.
     *
     * @return a {@link Trigger} active when POV is at 0 degrees
     */
    public Trigger getPovUp() {
        int pov = controller.getPOV();
        if (pov == 0) {
            return new Trigger(() -> true);
        }
        return new Trigger(() -> false);
    }

    /**
     * Returns a trigger for the POV Down direction.
     *
     * @return a {@link Trigger} active when POV is at 180 degrees
     */
    public Trigger getPovDown() {
        int pov = controller.getPOV();
        if (pov == 180) {
            return new Trigger(() -> true);
        }
        return new Trigger(() -> false);
    }

    /**
     * Returns a trigger for the POV Left direction.
     *
     * @return a {@link Trigger} active when POV is at 90 degrees
     */
    public Trigger getPovLeft() {
        int pov = controller.getPOV();
        if (pov == 90) {
            return new Trigger(() -> true);
        }
        return new Trigger(() -> false);
    }

    /**
     * Returns a trigger for the POV Right direction.
     *
     * @return a {@link Trigger} active when POV is at 270 degrees
     */
    public Trigger getPovRight() {
        int pov = controller.getPOV();
        if (pov == 270) {
            return new Trigger(() -> true);
        }
        return new Trigger(() -> false);
    }
}
