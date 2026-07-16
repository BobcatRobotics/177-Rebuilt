package org.bobcatrobotics.Controllers;

import org.bobcatrobotics.Controllers.Gamepads.ControllerBase;
import org.bobcatrobotics.Controllers.Gamepads.EightBitDoController;
import org.bobcatrobotics.Controllers.Gamepads.GenericHIDController;
import org.bobcatrobotics.Controllers.Gamepads.PS4Controller;
import org.bobcatrobotics.Controllers.Gamepads.PS5Controller;
import org.bobcatrobotics.Controllers.Gamepads.XboxController;
import org.bobcatrobotics.Controllers.Joysticks.GenericHIDJoystick;
import org.bobcatrobotics.Controllers.Joysticks.JoystickBase;
import org.bobcatrobotics.Controllers.Joysticks.LogitechJoystick;
import org.bobcatrobotics.Controllers.Joysticks.RuffyJoystick;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Utility class for automatic detection and creation of controllers and joysticks.
 * <p>
 * Detects connected USB devices via {@link DriverStation#getJoystickName(int)} and
 * returns an appropriate wrapper class instance for gamepads and joysticks.
 * Falls back to generic controllers if the type is unknown.
 * </p>
 */
public class ControllerAutoDetect {

    /**
     * Automatically detects the type of gamepad connected on a given port and
     * returns a corresponding {@link ControllerBase} instance.
     *
     * @param port           the USB port number of the controller
     * @param controllerName the display name for logging and alerts
     * @return a controller instance corresponding to the detected type
     *         (Xbox, PS4, PS5, 8BitDo, or GenericHIDController as fallback)
     */
    public static ControllerBase createGamepad(int port, String controllerName) {

        String name = DriverStation.getJoystickName(port).toLowerCase();
        boolean isXbox = DriverStation.getJoystickIsXbox(port);

        System.out.println("[ControllerDetect] Port " + port + " -> " + name);

        // XBOX detection
        if (isXbox || name.contains("xbox")) {
            System.out.println("[ControllerDetect] Detected XBOX controller.");
            return new XboxController(port, controllerName);
        }

        // PS4 detection
        if (name.contains("ps4") || name.contains("sony") || name.contains("wireless controller")) {
            System.out.println("[ControllerDetect] Detected PS4 controller.");
            return new PS4Controller(port, controllerName);
        }

        // PS5 detection
        if (name.contains("ps5") || name.contains("dualsense")) {
            System.out.println("[ControllerDetect] Detected PS5 controller.");
            return new PS5Controller(port, controllerName);
        }

        // 8BitDo detection
        if (name.contains("8bitdo") || name.contains("pro 2") || name.contains("sn30")) {
            System.out.println("[ControllerDetect] Detected 8BitDo controller.");
            return new EightBitDoController(port, controllerName);
        }

        // Fallback
        System.out.println("[ControllerDetect] Unknown controller type. Using GenericHIDController.");
        return new GenericHIDController(port, controllerName);
    }

    /**
     * Automatically detects the type of joystick connected on a given port and
     * returns a corresponding {@link JoystickBase} instance.
     *
     * @param port         the USB port number of the joystick
     * @param joystickName the display name for logging and alerts
     * @return a joystick instance corresponding to the detected type
     *         (Ruffy, Logitech, or GenericHIDJoystick as fallback)
     */
    public static JoystickBase createJoystick(int port, String joystickName) {

        String name = DriverStation.getJoystickName(port).toLowerCase();

        System.out.println("[ControllerDetect] Port " + port + " -> " + name);

        // Ruffy joystick detection
        if (name.contains("ruffy")) {
            System.out.println("[ControllerDetect] Detected ruffy joystick.");
            return new RuffyJoystick(port, joystickName);
        }

        // Logitech joystick detection
        if (name.contains("logitech")) {
            System.out.println("[ControllerDetect] Detected logitech joystick.");
            return new LogitechJoystick(port, joystickName);
        }

        // Fallback
        System.out.println("[ControllerDetect] Unknown controller type. Using GenericHIDController.");
        return new GenericHIDJoystick(port, joystickName);
    }
}
