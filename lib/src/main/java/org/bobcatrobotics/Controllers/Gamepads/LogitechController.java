package org.bobcatrobotics.Controllers.Gamepads;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Wrapper class for a Logitech-style game controller.
 * <p>
 * This class provides named accessors for buttons, axes, bumpers,
 * and POV directions using WPILib's {@link CommandJoystick} API.
 * Button names are mapped internally to hardware button indices.
 * </p>
 */
public class LogitechController {

    /** Underlying WPILib command-based joystick. */
    private final CommandJoystick controller;

    /** Mapping of human-readable button names to button indices. */
    private Map<String, Integer> buttonMap = new HashMap<>();

    /** Alert displayed when the controller is disconnected. */
    private final Alert controllerUnpluggedAlert;

    /**
     * Creates a new Logitech controller wrapper.
     *
     * @param port the USB port the controller is connected to
     * @param name a human-readable name used for alerts and logging
     */
    public LogitechController(int port, String name) {
        controllerUnpluggedAlert =
                new Alert(
                        name + " Joystick unplugged!",
                        AlertType.kWarning
                );

        controller = new CommandJoystick(port);

        buttonMap =
                Map.ofEntries(
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
     * @return a {@link Trigger} that becomes active when the button is pressed
     */
    public Trigger getButton(String buttonName) {
        return controller.button(buttonMap.get(buttonName).intValue());
    }

    /**
     * Returns the left trigger button.
     * <p>
     * This controller does not support analog trigger buttons, so this
     * trigger is always inactive.
     * </p>
     *
     * @return an inactive {@link Trigger}
     */
    public Trigger getLeftTrigger() {
        return new Trigger(() -> false);
    }

    /**
     * Returns the right trigger button.
     * <p>
     * This controller does not support analog trigger buttons, so this
     * trigger is always inactive.
     * </p>
     *
     * @return an inactive {@link Trigger}
     */
    public Trigger getRightTrigger() {
        return new Trigger(() -> false);
    }

    /**
     * Returns the left bumper button.
     *
     * @return a {@link Trigger} for the left bumper
     */
    public Trigger getLeftBumper() {
        return controller.button(5);
    }

    /**
     * Returns the right bumper button.
     *
     * @return a {@link Trigger} for the right bumper
     */
    public Trigger getRightBumper() {
        return controller.button(6);
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
        return controller.getRawAxis(4);
    }

    /**
     * Returns the Y-axis value of the left stick.
     * <p>
     * The value is inverted to match standard forward-positive conventions.
     * </p>
     *
     * @return left stick Y-axis value
     */
    public double getLeftY() {
        return -controller.getRawAxis(1);
    }

    /**
     * Returns the Y-axis value of the right stick.
     * <p>
     * The value is inverted to match standard forward-positive conventions.
     * </p>
     *
     * @return right stick Y-axis value
     */
    public double getRightY() {
        return -controller.getRawAxis(5);
    }

    /**
     * Updates controller connection alerts.
     * <p>
     * Should be called periodically to reflect the controller's
     * connection status.
     * </p>
     */
    public void updateControllerAlerts() {
        controllerUnpluggedAlert.set(!controller.isConnected());
    }

    /**
     * Returns a trigger for the POV up direction.
     *
     * @return a {@link Trigger} that activates when POV is up
     */
    public Trigger getPovUp() {
        return controller.povUp();
    }

    /**
     * Returns a trigger for the POV down direction.
     *
     * @return a {@link Trigger} that activates when POV is down
     */
    public Trigger getPovDown() {
        return controller.povDown();
    }

    /**
     * Returns a trigger for the POV left direction.
     *
     * @return a {@link Trigger} that activates when POV is left
     */
    public Trigger getPovLeft() {
        return controller.povLeft();
    }

    /**
     * Returns a trigger for the POV right direction.
     *
     * @return a {@link Trigger} that activates when POV is right
     */
    public Trigger getPovRight() {
        return controller.povRight();
    }
}
