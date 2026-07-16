package org.bobcatrobotics.Controllers.Gamepads;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Implementation of a PS5 controller.
 * <p>
 * Provides access to buttons, triggers, bumpers, sticks, and D-pad (POV)
 * with alert handling for controller disconnection.
 * </p>
 */
public class PS5Controller implements ControllerBase {

    /** Underlying WPILib PS5Controller instance. */
    private final edu.wpi.first.wpilibj.PS5Controller controller;

    /** Mapping of button names to BooleanSuppliers representing their state. */
    private Map<String, BooleanSupplier> buttonMap = new HashMap<>();

    /** Alert displayed when the controller is unplugged. */
    private final Alert controllerUnpluggedAlert;

    /**
     * Constructs a new PS5Controller.
     *
     * @param port the port the controller is connected to
     * @param name the name of the controller, used in alert messages
     */
    public PS5Controller(int port, String name) {
        controllerUnpluggedAlert =
                new Alert(name + " Joystick unplugged!", AlertType.kWarning);
        controller = new edu.wpi.first.wpilibj.PS5Controller(port);
        buttonMap = Map.ofEntries(
                Map.entry("Square", () -> controller.getSquareButton()),
                Map.entry("Cross", () -> controller.getCrossButton()),
                Map.entry("Circle", () -> controller.getCircleButton()),
                Map.entry("Triangle", () -> controller.getTriangleButton())
                // Additional buttons can be added here if needed
        );
    }

    /**
     * Returns a Trigger corresponding to the given button name.
     *
     * @param buttonName the name of the button
     * @return Trigger for the button
     */
    public Trigger getButton(String buttonName) {
        return new Trigger(buttonMap.get(buttonName));
    }

    /**
     * Returns a Trigger for the left trigger (L2) button.
     *
     * @return left trigger as a Trigger
     */
    public Trigger getLeftTrigger() {
        return new Trigger(() -> controller.getL2Button());
    }

    /**
     * Returns a Trigger for the right trigger (R2) button.
     *
     * @return right trigger as a Trigger
     */
    public Trigger getRightTrigger() {
        return new Trigger(() -> controller.getR2Button());
    }

    /**
     * Returns a Trigger for the left bumper (L1) button.
     *
     * @return left bumper as a Trigger
     */
    public Trigger getLeftBumper() {
        return new Trigger(() -> controller.getL1Button());
    }

    /**
     * Returns a Trigger for the right bumper (R1) button.
     *
     * @return right bumper as a Trigger
     */
    public Trigger getRightBumper() {
        return new Trigger(() -> controller.getR1Button());
    }

    /**
     * Returns the left stick X-axis value.
     *
     * @return left stick X-axis value
     */
    public double getLeftX() {
        return controller.getLeftX();
    }

    /**
     * Returns the right stick X-axis value.
     *
     * @return right stick X-axis value
     */
    public double getRightX() {
        return controller.getRightX();
    }

    /**
     * Returns the left stick Y-axis value.
     *
     * @return left stick Y-axis value
     */
    public double getLeftY() {
        return controller.getLeftY();
    }

    /**
     * Returns the right stick Y-axis value.
     *
     * @return right stick Y-axis value
     */
    public double getRightY() {
        return controller.getRightY();
    }

    /**
     * Updates the controller unplugged alert based on connection status.
     */
    public void updateControllerAlerts() {
        controllerUnpluggedAlert.set(!controller.isConnected());
    }

    /**
     * Returns a Trigger for when the POV (D-pad) is pressed up.
     *
     * @return POV up as a Trigger
     */
    public Trigger getPovUp() {
        int pov = controller.getPOV();
        if (pov == 0) {
            return new Trigger(() -> true);
        }
        return new Trigger(() -> false);
    }

    /**
     * Returns a Trigger for when the POV (D-pad) is pressed down.
     *
     * @return POV down as a Trigger
     */
    public Trigger getPovDown() {
        int pov = controller.getPOV();
        if (pov == 180) {
            return new Trigger(() -> true);
        }
        return new Trigger(() -> false);
    }

    /**
     * Returns a Trigger for when the POV (D-pad) is pressed left.
     *
     * @return POV left as a Trigger
     */
    public Trigger getPovLeft() {
        int pov = controller.getPOV();
        if (pov == 90) {
            return new Trigger(() -> true);
        }
        return new Trigger(() -> false);
    }

    /**
     * Returns a Trigger for when the POV (D-pad) is pressed right.
     *
     * @return POV right as a Trigger
     */
    public Trigger getPovRight() {
        int pov = controller.getPOV();
        if (pov == 270) {
            return new Trigger(() -> true);
        }
        return new Trigger(() -> false);
    }
}
