package org.bobcatrobotics.Controllers.Joysticks;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Wrapper class for a generic Logitech-style joystick, providing
 * access to raw buttons, stick axes, fine control axes, and
 * controller connection alerts.
 * <p>
 * Implements {@link JoystickBase} to standardize interaction with joysticks.
 * </p>
 */
public class LogitechJoystick implements JoystickBase {

    private CommandJoystick controller;
    private final Alert joystickUnpluggedAlert;
    private Map<String, Integer> buttonMap = new HashMap<>();

    /**
     * Constructs a LogitechJoystick wrapper for the given USB port and name.
     *
     * @param port the USB port of the joystick
     * @param name the display name of the joystick for alerts
     */
    public LogitechJoystick(int port, String name) {
        joystickUnpluggedAlert =
                new Alert(name + " Joystick unplugged!", AlertType.kWarning);
        controller = new CommandJoystick(port);
        buttonMap = Map.ofEntries(
                Map.entry("A", 1)
        );
    }

    /**
     * Returns a {@link Trigger} for a specified button by name.
     *
     * @param buttonName the name of the button
     * @return a Trigger that is active while the button is pressed
     */
    public Trigger getRawButton(String buttonName) {
        return controller.button(buttonMap.get(buttonName).intValue());
    }

    /**
     * Gets the X-axis value of the joystick.
     *
     * @return horizontal position of the joystick, range [-1.0, 1.0]
     */
    public double getStickX() {
        return controller.getRawAxis(0);
    }

    /**
     * Gets the Y-axis value of the joystick.
     *
     * @return vertical position of the joystick, range [-1.0, 1.0]
     */
    public double getStickY() {
        return controller.getRawAxis(1);
    }

    /**
     * Gets the Z-axis (twist) value of the joystick.
     *
     * @return twist position of the joystick, range [-1.0, 1.0]
     */
    public double getStickZ() {
        return controller.getRawAxis(2);
    }

    /**
     * Gets the fine control X-axis value.
     * Typically used for precision movements.
     *
     * @return fine control horizontal axis value
     */
    public double getFineStickX() {
        return controller.getRawAxis(0);
    }

    /**
     * Gets the fine control Y-axis value.
     * Typically used for precision movements.
     *
     * @return fine control vertical axis value
     */
    public double getFineStickY() {
        return controller.getRawAxis(2);
    }

    /**
     * Gets the fine control Z-axis value.
     * Typically used for precision movements.
     *
     * @return fine control twist axis value
     */
    public double getFineStickZ() {
        return controller.getRawAxis(2);
    }

    /**
     * Updates the joystick connection alert, notifying if the joystick is unplugged.
     */
    public void updateControllerAlerts() {
        joystickUnpluggedAlert.set(!controller.isConnected());
    }
}
