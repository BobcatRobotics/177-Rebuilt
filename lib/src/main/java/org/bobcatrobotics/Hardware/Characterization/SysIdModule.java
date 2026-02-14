package org.bobcatrobotics.Hardware.Characterization;

import java.util.EnumMap;
import java.util.Map;
import java.util.function.DoubleConsumer;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysIdModule {

    public enum Test {
        QF, QR, DF, DR
    }

    private final SysIdRoutine routine;
    private final DoubleConsumer voltageConsumer;
    private final Subsystem subsystem;

    public SysIdModule(
            String logPath,
            Subsystem subsystem,
            DoubleConsumer voltageConsumer,
            SysIdRoutine.Config config
    ) {
        this.subsystem = subsystem;
        this.voltageConsumer = voltageConsumer;

        routine = new SysIdRoutine(
                config,
                new SysIdRoutine.Mechanism(
                        (Voltage voltage) ->
                                voltageConsumer.accept(voltage.in(Units.Volts)),
                        null,
                        subsystem
                )
        );
    }

    /** Stop output safely */
    private void stop() {
        voltageConsumer.accept(0.0);
    }

    /** Quasistatic */
    public Command quasistatic(SysIdRoutine.Direction direction) {
        return Commands.runOnce(this::stop)
                .andThen(routine.quasistatic(direction));
    }

    /** Dynamic */
    public Command dynamic(SysIdRoutine.Direction direction) {
        return Commands.runOnce(this::stop)
                .andThen(routine.dynamic(direction));
    }

    /** Generate all 4 tests */
    public Map<Test, Command> generateAllTests() {
        Map<Test, Command> tests = new EnumMap<>(Test.class);

        tests.put(Test.QF, quasistatic(SysIdRoutine.Direction.kForward));
        tests.put(Test.QR, quasistatic(SysIdRoutine.Direction.kReverse));
        tests.put(Test.DF, dynamic(SysIdRoutine.Direction.kForward));
        tests.put(Test.DR, dynamic(SysIdRoutine.Direction.kReverse));

        return tests;
    }

    public void getResults(){
    }
}