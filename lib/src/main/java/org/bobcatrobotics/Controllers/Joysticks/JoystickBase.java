package org.bobcatrobotics.Controllers.Joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Common interface for joystick-based input devices.
 * <p>
 * This interface defines a hardware-agnostic abstraction over different
 * joystick models, providing standardized access to axes, buttons, and
 * controller-specific alerts. Implementations are responsible for mapping
 * physical inputs to these logical accessors.
 * </p>
 */
public interface JoystickBase {

    /**
     * Returns a {@link Trigger} associated with a named button.
     * <p>
     * Button names are implementation-defined and typically correspond
     * to a logical or labeled button on the physical joystick (e.g.
     * "Trigger", "Thumb", "Button1").
     * </p>
     *
     * @param buttonName the logical name of the button
     * @return a {@link Trigger} that becomes active while the button is pressed
     */
    Trigger getRawButton(String buttonName);

    /**
     * Returns the primary X-axis value of the joystick.
     *
     * @return the X-axis value, typically in the range [-1.0, 1.0]
     */
    double getStickX();

    /**
     * Returns the primary Y-axis value of the joystick.
     *
     * @return the Y-axis value, typically in the range [-1.0, 1.0]
     */
    double getStickY();

    /**
     * Returns the primary Z-axis (twist or throttle) value of the joystick.
     *
     * @return the Z-axis value, typically in the range [-1.0, 1.0]
     */
    double getStickZ();

    /**
     * Returns the fine-control X-axis value of the joystick.
     * <p>
     * Fine axes are typically scaled or filtered versions of the primary
     * axes to allow for more precise control.
     * </p>
     *
     * @return the fine-control X-axis value
     */
    double getFineStickX();

    /**
     * Returns the fine-control Y-axis value of the joystick.
     *
     * @return the fine-control Y-axis value
     */
    double getFineStickY();

    /**
     * Returns the fine-control Z-axis value of the joystick.
     *
     * @return the fine-control Z-axis value
     */
    double getFineStickZ();

    /**
     * Updates any controller-specific alerts or status indicators.
     * <p>
     * This method is intended to be called periodically (e.g. once per
     * robot loop) to handle rumble feedback, LED indicators, or other
     * operator alerts supported by the joystick hardware.
     * </p>
     */
    void updateControllerAlerts();
}
