package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

public final class DebouncedCommand {
    private static final Debouncer atSpeedDebouncer = new Debouncer(1.0, DebounceType.kRising);

    public static Command debouncer(Command cmd, Timer timer, double time, BooleanSupplier isAtSpeed) {
        return cmd.beforeStarting(() -> timer.reset())
                .until(() -> {
                    if (isAtSpeed.getAsBoolean()) {
                        if (!timer.isRunning()) {
                            timer.start();
                        }
                    } else {
                        timer.stop();
                        timer.reset();
                    }
                    return timer.hasElapsed(time);
                });
    }

    public static Command debouncer(Command cmd, double time, BooleanSupplier isAtSpeed) {
        atSpeedDebouncer.setDebounceTime(time);
        return cmd.until(() -> atSpeedDebouncer.calculate(isAtSpeed.getAsBoolean()));
    }
}
