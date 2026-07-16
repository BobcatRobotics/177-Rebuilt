# FRC Led Controller Library

A simple, elegant LED animation library for led controll devices with built-in simulation support.

## ✨ Features

- 🎨 **2 Animation Types** - Breathing, Blink
- 🖥️ **Simulation GUI** - Develop and test LED code without hardware
- 📦 **Strip Segmentation** - Control multiple independent sections
- 🎮 **WPILib Command Integration** - Works seamlessly with command-based robot code
- 📚 **Comprehensive JavaDocs** - Full documentation for every method
- 🔧 **Type-Safe Color System** - Predefined colors plus custom RGB support

### Basic Usage

```java
import frc.robot.CANdle.CANdleLib;
import frc.robot.CANdle.CANdleLib.*;
import com.ctre.phoenix.led.CANdle;

public class RobotContainer {
    // Initialize the library
    private final LedStrip ledStrip = new LedStrip(8, 0, 7);
    private final LedControllerIO candle = new CANDleIOReal(0, ledStrip, StripTypeValue.RGBW);
    private final LedController leds = new LedController(candle);
    
    public RobotContainer(){
        // Set a solid color
        leds.setLEDs(255,255,255,0,ledStrip.startIdx,ledStrip.endIdx);

        // Breathe Animation color with 60s Timeout
        var animateBreathingLedTimeout = new Breathe(candle, new Color(255,255,255));
        Command test = animateBreathingLedTimeout.withTimeout(60);
        test.schedule();
        // Breathe Animation color with untill condition
        BooleanSupplier condition = new CommandXboxController(0).a();
        var animateBreathingUntil = new Breathe(candle, new Color(255,255,255));
        Command test = animateBreathingUntil.untill( condition );
        test.schedule();

        // Blink Animation color with 60s Timeout
        var animateBlinkLedTimeout = new Blink(candle, new Color(255,255,255));
        Command test = animateBlinkLedTimeout.withTimeout(60);
        test.schedule();
        // Blink Animation color with untill condition
        BooleanSupplier condition = new CommandXboxController(0).a();
        var animateBlinkLedUntil = new Blink(candle, new Color(255,255,255));
        Command test = animateBlinkLedUntil.untill( condition );
        test.schedule();
    }
}
```