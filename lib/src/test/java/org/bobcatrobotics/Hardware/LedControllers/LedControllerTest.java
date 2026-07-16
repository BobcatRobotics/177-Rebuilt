package org.bobcatrobotics.Hardware.LedControllers;

import org.junit.jupiter.api.Test;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static org.junit.jupiter.api.Assertions.*;
import java.util.function.BooleanSupplier;
import org.bobcatrobotics.Hardware.LedControllers.Animations.Breathe;
import org.bobcatrobotics.Hardware.LedControllers.Animations.Blink;

public class LedControllerTest {


    public LedControllerTest() {

    }

    @Test
    void testFullConstructor() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        LedControllerIO candle = new LedSim(0, new LedStrip(8, 0, 7), StripTypeValue.RGBW);
        LedController leds = new LedController(candle);
        assertEquals(8, leds.getCount());
    }

    @Test
    void testBreatheWithTimeoutAnimation() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        LedControllerIO candle = new LedSim(0, new LedStrip(8, 0, 7), StripTypeValue.RGBW);
        LedController leds = new LedController(candle);
        assertEquals(8, leds.getCount());

        var animateLed = new Breathe(candle, new Color(255, 255, 255));
        assertEquals(false, animateLed.isFinished);
        Command test = animateLed.withTimeout(60);

        // Run for 0.5 seconds → should still be running
        advanceTime(scheduler, 0.5);
        assertFalse(animateLed.isRunning(), "Command ended too early");

        // Run past 2.0 seconds → should finish
        advanceTime(scheduler, 61);
        assertTrue(!animateLed.isRunning(), "Command did not end after timeout");
    }

    @Test
    void testBreatheUntilAnimation() {
        BooleanSupplier condition = () -> true;
        LedControllerIO candle = new LedSim(0, new LedStrip(8, 0, 7), StripTypeValue.RGBW);
        LedController leds = new LedController(candle);
        assertEquals(8, leds.getCount());

        var animateLed = new Breathe(candle, new Color(255, 255, 255));
        assertEquals(false, animateLed.isFinished);
        Command test = animateLed.untill(condition);
        CommandScheduler scheduler = CommandScheduler.getInstance();
        // Run for 0.5 seconds → should still be running
        advanceTime(scheduler, 0.5);
        assertFalse(animateLed.isRunning(), "Command ended too early");

        // Run past 2.0 seconds → should finish
        advanceTime(scheduler, 15);
        assertFalse(animateLed.isRunning(), "Command did not end after timeout");

        // Run past 2.0 seconds → should finish
        advanceTime(scheduler, 15);
        condition = () -> true;
        assertTrue(!animateLed.isRunning(), "Command did not end after timeout");
    }


    @Test
    void testBlinkWithTimeoutAnimation() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        LedControllerIO candle = new LedSim(0, new LedStrip(8, 0, 7), StripTypeValue.RGBW);
        LedController leds = new LedController(candle);
        assertEquals(8, leds.getCount());

        var animateLed = new Blink(candle, new Color(255, 255, 255), 5);
        assertEquals(false, animateLed.isFinished);
        Command test = animateLed.withTimeout(60);
        // Run for 0.5 seconds → should still be running
        advanceTime(scheduler, 0.5);
        assertFalse(animateLed.isRunning(), "Command ended too early");

        // Run past 2.0 seconds → should finish
        advanceTime(scheduler, 61);
        assertTrue(!animateLed.isRunning(), "Command did not end after timeout");
    }

    @Test
    void testBlinkUntilAnimation() {
        BooleanSupplier condition = () -> true;
        CommandScheduler scheduler = CommandScheduler.getInstance();
        LedControllerIO candle = new LedSim(0, new LedStrip(8, 0, 7), StripTypeValue.RGBW);
        LedController leds = new LedController(candle);
        assertEquals(8, leds.getCount());

        var animateLed = new Blink(candle, new Color(255, 255, 255), 5);
        assertEquals(false, animateLed.isFinished);
        Command test = animateLed.untill(condition);
        // Run for 0.5 seconds → should still be running
        advanceTime(scheduler, 0.5);
        assertFalse(animateLed.isRunning(), "Command ended too early");

        // Run past 5.0 seconds → should finish
        advanceTime(scheduler, 5);
        assertFalse(animateLed.isRunning(), "Command did not end after timeout");

        // Run past 10.0 seconds → should finish
        advanceTime(scheduler, 10);
        condition = () -> true;
        assertTrue(!animateLed.isRunning(), "Command did not end after timeout");
    }

    private void advanceTime(CommandScheduler scheduler, double seconds) {
        final double step = 0.02; // 20ms like real robot
        int cycles = (int) Math.ceil(seconds / step);

        for (int i = 0; i < cycles; i++) {
            SimHooks.stepTiming(step);
            scheduler.run();
        }
    }
}
