package org.bobcatrobotics.Controllers.Gamepads;

/**
 * Factory class for creating various gamepad and controller instances.
 * <p>
 * Provides static methods to instantiate commonly used controllers by
 * specifying the port and a friendly name for alert purposes.
 * </p>
 */
public class ControllerFactory {

    /**
     * Creates an Xbox controller instance.
     *
     * @param port the port the controller is connected to
     * @param name the name of the controller, used in alerts
     * @return a new XboxController implementing ControllerBase
     */
    public static ControllerBase xbox(int port, String name) {
        return new XboxController(port, name);
    }

    /**
     * Creates a PS4 controller instance.
     *
     * @param port the port the controller is connected to
     * @param name the name of the controller, used in alerts
     * @return a new PS4Controller implementing ControllerBase
     */
    public static ControllerBase ps4(int port, String name) {
        return new PS4Controller(port, name);
    }

    /**
     * Creates a PS5 controller instance.
     *
     * @param port the port the controller is connected to
     * @param name the name of the controller, used in alerts
     * @return a new PS5Controller implementing ControllerBase
     */
    public static ControllerBase ps5(int port, String name) {
        return new PS5Controller(port, name);
    }

    /**
     * Creates an EightBitDo controller instance.
     *
     * @param port the port the controller is connected to
     * @param name the name of the controller, used in alerts
     * @return a new EightBitDoController implementing ControllerBase
     */
    public static ControllerBase eightBitDo(int port, String name) {
        return new EightBitDoController(port, name);
    }
}
