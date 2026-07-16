package org.bobcatrobotics.Controllers.Gamepads;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Interface defining a standardized controller for gamepads and joysticks.
 * <p>
 * Provides methods for reading analog stick values, button presses, triggers,
 * bumpers, and POV (D-pad) directions, as well as updating any controller
 * connection alerts.
 * </p>
 */
public interface ControllerBase {

    /**
     * Gets the X-axis value of the left stick.
     *
     * @return a double representing the horizontal position of the left stick
     */
    double getLeftX();

    /**
     * Gets the Y-axis value of the left stick.
     *
     * @return a double representing the vertical position of the left stick
     */
    double getLeftY();

    /**
     * Gets the X-axis value of the right stick.
     *
     * @return a double representing the horizontal position of the right stick
     */
    double getRightX();

    /**
     * Gets the Y-axis value of the right stick.
     *
     * @return a double representing the vertical position of the right stick
     */
    double getRightY();

    /**
     * Gets a trigger representing a named button on the controller.
     *
     * @param buttonName the name of the button (e.g., "A", "B", "X", "Y")
     * @return a Trigger that is active while the button is pressed
     */
    Trigger getButton(String buttonName);

    /**
     * Gets a trigger representing the left analog trigger.
     *
     * @return a Trigger that is active while the left trigger is pressed
     */
    Trigger getLeftTrigger();

    /**
     * Gets a trigger representing the right analog trigger.
     *
     * @return a Trigger that is active while the right trigger is pressed
     */
    Trigger getRightTrigger();

    /**
     * Gets a trigger representing the left bumper.
     *
     * @return a Trigger that is active while the left bumper is pressed
     */
    Trigger getLeftBumper();

    /**
     * Gets a trigger representing the right bumper.
     *
     * @return a Trigger that is active while the right bumper is pressed
     */
    Trigger getRightBumper();

    /**
     * Gets a trigger representing the D-pad up direction.
     *
     * @return a Trigger that is active while the POV is pressed up
     */
    Trigger getPovUp();

    /**
     * Gets a trigger representing the D-pad down direction.
     *
     * @return a Trigger that is active while the POV is pressed down
     */
    Trigger getPovDown();

    /**
     * Gets a trigger representing the D-pad left direction.
     *
     * @return a Trigger that is active while the POV is pressed left
     */
    Trigger getPovLeft();

    /**
     * Gets a trigger representing the D-pad right direction.
     *
     * @return a Trigger that is active while the POV is pressed right
     */
    Trigger getPovRight();

    /**
     * Updates any alerts related to the controller, such as connection status.
     * <p>
     * This is typically called periodically to notify users if the controller
     * has been disconnected.
     * </p>
     */
    void updateControllerAlerts();
}
