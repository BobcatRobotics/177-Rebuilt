package org.bobcatrobotics.Controllers.Joysticks;

/**
 * Factory class for creating {@link JoystickBase} implementations.
 * <p>
 * This class centralizes joystick construction logic and provides
 * named factory methods for supported joystick hardware. Using a
 * factory allows joystick types to be swapped or extended without
 * modifying calling code.
 * </p>
 */
public class JoystickFactory {

    /**
     * Creates a Logitech joystick instance.
     *
     * @param port the USB port the joystick is connected to (as configured in the Driver Station)
     * @param name a human-readable name used for identification and logging
     * @return a {@link JoystickBase} representing a Logitech joystick
     */
    public static JoystickBase logitech(int port, String name) {
        return new LogitechJoystick(port, name);
    }

    /**
     * Creates a Ruffy joystick instance.
     *
     * @param port the USB port the joystick is connected to (as configured in the Driver Station)
     * @param name a human-readable name used for identification and logging
     * @return a {@link JoystickBase} representing a Ruffy joystick
     */
    public static JoystickBase ruffy(int port, String name) {
        return new RuffyJoystick(port, name);
    }
}
