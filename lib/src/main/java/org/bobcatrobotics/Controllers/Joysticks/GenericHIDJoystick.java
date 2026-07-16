package org.bobcatrobotics.Controllers.Joysticks;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Generic {@link JoystickBase} implementation backed by WPILib's {@link GenericHID}.
 * <p>
 * This class provides a simple, hardware-agnostic joystick wrapper that maps
 * named buttons and axes to raw HID inputs. It also exposes connection status
 * alerts to inform operators when the joystick becomes disconnected.
 * </p>
 */
public class GenericHIDJoystick implements JoystickBase {

    /** Underlying WPILib GenericHID device. */
    private final GenericHID controller;

    /** Alert displayed when the joystick becomes disconnected. */
    private final Alert joystickUnpluggedAlert;

    /**
     * Mapping between logical button names and raw button indices.
     * <p>
     * Button indices follow WPILib's 1-based HID button numbering.
     * </p>
     */
    private Map<String, Integer> buttonMap = new HashMap<>();

    /**
     * Creates a new {@code GenericHIDJoystick}.
     *
     * @param port the USB port the joystick is connected to (as configured in the Driver Station)
     * @param name a human-readable name used for alerts and diagnostics
     */
    public GenericHIDJoystick(int port, String name) {
        joystickUnpluggedAlert =
                new Alert(name + " Joystick unplugged!", AlertType.kWarning);

        controller = new GenericHID(port);

        buttonMap = Map.ofEntries(
                Map.entry("A", 1)
        );
    }

    /**
     * Returns a {@link Trigger} for a named joystick button.
     *
     * @param buttonName the logical name of the button (as defined in the button map)
     * @return a trigger that is active while the button is pressed
     * @throws NullPointerException if the button name is not present in the map
     */
    @Override
    public Trigger getRawButton(String buttonName) {
        return new Trigger(
                () -> controller.getRawButton(buttonMap.get(buttonName).intValue())
        );
    }

    /**
     * Returns the primary X-axis value of the joystick.
     *
     * @return the X-axis value, typically in the range [-1.0, 1.0]
     */
    @Override
    public double getStickX() {
        return controller.getRawAxis(1);
    }

    /**
     * Returns the primary Y-axis value of the joystick.
     *
     * @return the Y-axis value, typically in the range [-1.0, 1.0]
     */
    @Override
    public double getStickY() {
        return controller.getRawAxis(2);
    }

    /**
     * Returns the primary Z-axis (twist or throttle) value of the joystick.
     *
     * @return the Z-axis value, typically in the range [-1.0, 1.0]
     */
    @Override
    public double getStickZ() {
        return controller.getRawAxis(3);
    }

    /**
     * Returns the fine-control X-axis value.
     * <p>
     * This implementation currently mirrors the primary X-axis.
     * </p>
     *
     * @return the fine-control X-axis value
     */
    @Override
    public double getFineStickX() {
        return controller.getRawAxis(1);
    }

    /**
     * Returns the fine-control Y-axis value.
     * <p>
     * This implementation currently mirrors the primary Y-axis.
     * </p>
     *
     * @return the fine-control Y-axis value
     */
    @Override
    public double getFineStickY() {
        return controller.getRawAxis(2);
    }

    /**
     * Returns the fine-control Z-axis value.
     * <p>
     * This implementation currently mirrors the Y-axis.
     * </p>
     *
     * @return the fine-control Z-axis value
     */
    @Override
    public double getFineStickZ() {
        return controller.getRawAxis(2);
    }

    /**
     * Updates controller-related alerts.
     * <p>
     * This should be called periodically to ensure connection status
     * alerts remain accurate.
     * </p>
     */
    @Override
    public void updateControllerAlerts() {
        joystickUnpluggedAlert.set(!controller.isConnected());
    }
}
