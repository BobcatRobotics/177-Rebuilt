import java.util.function.DoubleConsumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysIdModule {

    private final SysIdRoutine routine;
    private final Runnable stop;

    public SysIdModule(
            String logPath,
            Subsystem subsystem,
            DoubleConsumer voltageConsumer,
            Runnable stop
    ) {

        this.stop = stop;

        routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        state -> Logger.recordOutput(logPath, state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        (Voltage voltage) ->
                                voltageConsumer.accept(voltage.in(Units.Volts)),
                        null,
                        subsystem
                )
        );
    }

    /** Quasistatic test command */
    public Command quasistatic(SysIdRoutine.Direction direction) {
        return Commands.run(stop).withTimeout(1.0)
                .andThen(routine.quasistatic(direction));
    }

    /** Dynamic test command */
    public Command dynamic(SysIdRoutine.Direction direction) {
        return Commands.run(stop).withTimeout(1.0)
                .andThen(routine.dynamic(direction));
    }
}
