package org.bobcatrobotics.Controllers.Gamepads;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * {@link ControllerBase} implementation for the 8BitDo game controller.
 * <p>
 * This class wraps a {@link CommandJoystick} and exposes a standardized
 * control layout for use in the command-based framework. Logical button
 * names are mapped to raw button indices, and controller connection
 * status is monitored via a driver alert.
 * </p>
 */
public class EightBitDoController implements ControllerBase {

    /** Underlying WPILib command-based joystick wrapper. */
    private final CommandJoystick controller;

    /** Alert displayed when the controller becomes disconnected. */
    private final Alert controllerUnpluggedAlert;

    /**
     * Mapping between logical button names and raw button indices.
     * <p>
     * Button indices follow WPILib's standard gamepad mapping.
     * </p>
     */
    private Map<String, Integer> buttonMap = new HashMap<>();

    /**
     * Creates a new {@code EightBitDoController}.
     *
     * @param port the USB port the controller is connected to (as configured in the Driver Station)
     * @param name a human-readable name used for alerts and diagnostics
     */
    public EightBitDoController(int port, String name) {
        controllerUnpluggedAlert =
                new Alert(name + " Joystick unplugged!", AlertType.kWarning);

        controller = new CommandJoystick(port);

        buttonMap = Map.ofEntries(
                Map.entry("A", 2),
                Map.entry("B", 1),
                Map.entry("X", 4),
                Map.entry("Y", 3),
                Map.entry("Select", 7),
                Map.entry("Start", 8)
        );
    }

    /**
     * Returns a {@link Trigger} for a named controller button.
     *
     * @param buttonName the logical name of the button (e.g. "A", "B", "X", "Y")
     * @return a trigger that is active while the button is pressed
     * @throws NullPointerException if the button name is not present in the map
     */
    @Override
    public Trigger getButton(String buttonName) {
        return controller.button(buttonMap.get(buttonName).intValue());
    }

    /**
     * Returns a trigger for the left trigger button.
     *
     * @return a trigger that is active while the left trigger is pressed
     */
    @Override
    public Trigger getLeftTrigger() {
        return controller.button(9);
    }

    /**
     * Returns a trigger for the right trigger button.
     *
     * @return a trigger that is active while the right trigger is pressed
     */
    @Override
    public Trigger getRightTrigger() {
        return controller.button(10);
    }

    /**
     * Returns a trigger for the left bumper button.
     *
     * @return a trigger that is active while the left bumper is pressed
     */
    @Override
    public Trigger getLeftBumper() {
        return controller.button(5);
    }

    /**
     * Returns a trigger for the right bumper button.
     *
     * @return a trigger that is active while the right bumper is pressed
     */
    @Override
    public Trigger getRightBumper() {
        return controller.button(6);
    }

    /**
     * Returns the X-axis value of the left analog stick.
     *
     * @return the left stick X-axis value, typically in the range [-1.0, 1.0]
     */
    @Override
    public double getLeftX() {
        return controller.getRawAxis(0);
    }

    /**
     * Returns the X-axis value of the right analog stick.
     *
     * @return the right stick X-axis value, typically in the range [-1.0, 1.0]
     */
    @Override
    public double getRightX() {
        return controller.getRawAxis(4);
    }

    /**
     * Returns the Y-axis value of the left analog stick.
     * <p>
     * The value is inverted to match common FRC forward-positive conventions.
     * </p>
     *
     * @return the left stick Y-axis value
     */
    @Override
    public double getLeftY() {
        return -controller.getRawAxis(1);
    }

    /**
     * Returns the Y-axis value of the right analog stick.
     * <p>
     * The value is inverted to match common FRC forward-positive conventions.
     * </p>
     *
     * @return the right stick Y-axis value
     */
    @Override
    public double getRightY() {
        return -controller.getRawAxis(3);
    }

    /**
     * Updates controller-related alerts.
     * <p>
     * This method should be called periodically to ensure the controller
     * connection status is accurately reflected.
     * </p>
     */
    @Override
    public void updateControllerAlerts() {
        controllerUnpluggedAlert.set(!controller.isConnected());
    }

    /**
     * Returns a trigger for the POV up direction.
     *
     * @return a trigger active while the POV is pressed upward
     */
    @Override
    public Trigger getPovUp() {
        return controller.povUp();
    }

    /**
     * Returns a trigger for the POV down direction.
     *
     * @return a trigger active while the POV is pressed downward
     */
    @Override
    public Trigger getPovDown() {
        return controller.povDown();
    }

    /**
     * Returns a trigger for the POV left direction.
     *
     * @return a trigger active while the POV is pressed left
     */
    @Override
    public Trigger getPovLeft() {
        return controller.povLeft();
    }

    /**
     * Returns a trigger for the POV right direction.
     *
     * @return a trigger active while the POV is pressed right
     */
    @Override
    public Trigger getPovRight() {
        return controller.povRight();
    }
}
