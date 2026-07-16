package org.bobcatrobotics.Hardware.Motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FindLimit {
    private final DutyCycleOut calibrationRequest;

    /** Trigger to detect when the motor drives into a hard stop. */
    public final Trigger isHardStop;

    private TalonFX[] motors;
    private boolean runClockwise;
    private Subsystem sub;
    public FindLimit(Subsystem sub, boolean runClockwise, TalonFX... motors) {
        this(runClockwise,motors);
        this.sub = sub;
    }
    public FindLimit(boolean runClockwise, TalonFX... motors) {
        this.motors = motors;
        this.runClockwise = runClockwise;
        double requestOutput = 0.0;
        if (runClockwise) {
            requestOutput = 0.1;
        } else {
            requestOutput = -0.1;
        }
        TalonFX leader = this.motors[0];
        isHardStop = new Trigger(() -> {
            return leader.getVelocity().getValue().abs(RotationsPerSecond) < 1 &&
                    leader.getTorqueCurrent().getValue().abs(Amps) > 10;
        }).debounce(0.1);
        calibrationRequest = new DutyCycleOut(requestOutput)
                .withIgnoreHardwareLimits(true)
                .withIgnoreSoftwareLimits(true);
    }

    /**
     * Seeks the hard stop. This slowly drives the motor up/down based on
     * initialization parameters.
     * until we see a drop in velocity and a spike in stator current,
     * indicating that we've hit a hard stop.
     *
     * @return Command to run
     */
    public Command findLimit(Subsystem requiredSubsystem, Runnable endBehaviour) {
        sub = requiredSubsystem;
        return Commands.run(() -> {
            for (int m = 0; m < motors.length; m++) {
                motors[m].setControl(calibrationRequest);
            }
        })
                .until(isHardStop)
                .andThen(
                        endBehaviour, sub);
    }

    /**
     * Seeks the hard stop. This slowly drives the motor up/down based on
     * initialization parameters.
     * until we see a drop in velocity and a spike in stator current,
     * indicating that we've hit a hard stop.
     *
     * @return Command to run
     */
    public Command findLimit() {
        return Commands.run(() -> {
            for (int m = 0; m < motors.length; m++) {
                motors[m].setControl(calibrationRequest);
            }
        })
                .until(isHardStop)
                .andThen(
                        () -> {
                            for (int m = 0; m < motors.length; m++) {
                                motors[m].setPosition(0.0);
                            }
                        });
    }
}
