package org.bobcatrobotics.Controllers.Gamepads;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Wrapper class for the WPILib CommandXboxController to provide
 * a standardized interface for buttons, triggers, bumpers, sticks,
 * and POV (D-pad) directions.
 * <p>
 * This class implements {@link ControllerBase} and adds alerts for
 * controller disconnection.
 * </p>
 */
public class XboxController implements ControllerBase {

    private final CommandXboxController controller;
    private Map<String, Integer> buttonMap = new HashMap<>();
    private final Alert controllerUnpluggedAlert;

    /**
     * Constructs an XboxController wrapper for the specified port and name.
     *
     * @param port the USB port of the Xbox controller
     * @param name the name of the controller used for alerts
     */
    public XboxController(int port, String name) {
        controllerUnpluggedAlert = new Alert(name + " Joystick unplugged!", AlertType.kWarning);
        controller = new CommandXboxController(port);
        buttonMap = Map.ofEntries(
            Map.entry("A", 1),
            Map.entry("B", 2),
            Map.entry("X", 3),
            Map.entry("Y", 4),
            Map.entry("LeftBumper", 5),
            Map.entry("RightBumper", 6),
            Map.entry("Back", 7),
            Map.entry("Start", 8)
        );
    }

    /**
     * Returns a {@link Trigger} for the specified button name.
     *
     * @param buttonName the name of the button (e.g., "A", "B", "X", "Y")
     * @return a Trigger active while the button is pressed
     */
    public Trigger getButton(String buttonName) {
        return controller.button(buttonMap.get(buttonName).intValue());
    }

    /**
     * Returns a Trigger for the left analog trigger with a 0.1 deadband.
     *
     * @return a Trigger active while the left trigger is pressed beyond 0.1
     */
    public Trigger getLeftTrigger() {
        return controller.leftTrigger(0.1);
    }

    /**
     * Returns a Trigger for the right analog trigger with a 0.1 deadband.
     *
     * @return a Trigger active while the right trigger is pressed beyond 0.1
     */
    public Trigger getRightTrigger() {
        return controller.rightTrigger(0.1);
    }

    /**
     * Returns a Trigger for the left bumper.
     *
     * @return a Trigger active while the left bumper is pressed
     */
    public Trigger getLeftBumper() {
        return controller.leftBumper();
    }

    /**
     * Returns a Trigger for the right bumper.
     *
     * @return a Trigger active while the right bumper is pressed
     */
    public Trigger getRightBumper() {
        return controller.rightBumper();
    }

    /**
     * Gets the X-axis value of the left stick.
     *
     * @return a double representing the horizontal position of the left stick
     */
    public double getLeftX() {
        return controller.getLeftX();
    }

    /**
     * Gets the X-axis value of the right stick.
     *
     * @return a double representing the horizontal position of the right stick
     */
    public double getRightX() {
        return controller.getRightX();
    }

    /**
     * Gets the Y-axis value of the left stick.
     *
     * @return a double representing the vertical position of the left stick
     */
    public double getLeftY() {
        return controller.getLeftY();
    }

    /**
     * Gets the Y-axis value of the right stick.
     *
     * @return a double representing the vertical position of the right stick
     */
    public double getRightY() {
        return controller.getRightY();
    }

    /**
     * Updates alerts for the controller, notifying if it is disconnected.
     */
    public void updateControllerAlerts() {
        controllerUnpluggedAlert.set(!controller.isConnected());
    }

    /**
     * Returns a Trigger for the POV (D-pad) up direction.
     *
     * @return a Trigger active while the POV is pressed up
     */
    public Trigger getPovUp() {
        return controller.povUp();
    }

    /**
     * Returns a Trigger for the POV (D-pad) down direction.
     *
     * @return a Trigger active while the POV is pressed down
     */
    public Trigger getPovDown() {
        return controller.povDown();
    }

    /**
     * Returns a Trigger for the POV (D-pad) left direction.
     *
     * @return a Trigger active while the POV is pressed left
     */
    public Trigger getPovLeft() {
        return controller.povLeft();
    }

    /**
     * Returns a Trigger for the POV (D-pad) right direction.
     *
     * @return a Trigger active while the POV is pressed right
     */
    public Trigger getPovRight() {
        return controller.povRight();
    }
}
