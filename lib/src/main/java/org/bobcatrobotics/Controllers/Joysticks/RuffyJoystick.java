package org.bobcatrobotics.Controllers.Joysticks;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Implementation of a joystick for the Ruffy controller.
 * <p>
 * Provides access to axis values, buttons, and handles controller unplugged alerts.
 * </p>
 */
public class RuffyJoystick implements JoystickBase {

    /** Underlying WPILib CommandJoystick instance. */
    private CommandJoystick controller;

    /** Alert displayed when the joystick is unplugged. */
    private final Alert joystickUnpluggedAlert;

    /** Mapping of button names to button numbers. */
    private Map<String, Integer> buttonMap = new HashMap<>();

    /**
     * Constructs a new RuffyJoystick.
     *
     * @param port the port the joystick is connected to
     * @param name the name of the joystick, used in alert messages
     */
    public RuffyJoystick(int port, String name) {
        joystickUnpluggedAlert =
                new Alert(name + " Joystick unplugged!", AlertType.kWarning);
        controller = new CommandJoystick(port);
        buttonMap = Map.ofEntries(
                Map.entry("A", 1)
        );
    }

    /**
     * Returns a Trigger corresponding to the given button name.
     *
     * @param buttonName the name of the button
     * @return Trigger for the button
     */
    public Trigger getRawButton(String buttonName) {
        return controller.button(buttonMap.get(buttonName).intValue());
    }

    /**
     * Returns the X-axis value of the main stick.
     *
     * @return stick X-axis value
     */
    public double getStickX() {
        return controller.getRawAxis(0);
    }

    /**
     * Returns the Y-axis value of the main stick.
     *
     * @return stick Y-axis value
     */
    public double getStickY() {
        return controller.getRawAxis(1);
    }

    /**
     * Returns the Z-axis value of the main stick.
     *
     * @return stick Z-axis value
     */
    public double getStickZ() {
        return controller.getRawAxis(2);
    }

    /**
     * Returns the fine X-axis value for precise control.
     *
     * @return fine stick X-axis value
     */
    public double getFineStickX() {
        return controller.getRawAxis(0);
    }

    /**
     * Returns the fine Y-axis value for precise control.
     *
     * @return fine stick Y-axis value
     */
    public double getFineStickY() {
        return controller.getRawAxis(2);
    }

    /**
     * Returns the fine Z-axis value for precise control.
     *
     * @return fine stick Z-axis value
     */
    public double getFineStickZ() {
        return controller.getRawAxis(2);
    }

    /**
     * Updates the controller alerts based on whether the joystick is connected.
     */
    public void updateControllerAlerts() {
        joystickUnpluggedAlert.set(!controller.isConnected());
    }
}
