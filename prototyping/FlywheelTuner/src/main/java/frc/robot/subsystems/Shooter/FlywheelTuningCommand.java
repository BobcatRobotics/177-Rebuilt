package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class FlywheelTuningCommand extends Command {

    private final FlywheelTunerSubsystem flywheel;

    public enum State {
        ZERO,
        HIGH_SETPOINT,
        FIND_KS_HIGH,
        INIT_KP,
        FIND_KV,
        LOW_SETPOINT,
        REFINE_KS,
        RETURN_HIGH,
        FIND_KP,
        VERIFY,
        DONE
    }

    private State state = State.ZERO;

    private final double maxVel;
    private double setpoint;

    private double kS = 0;
    private double kP = 0;
    private double kV = 0;

    private final double kS_step = 0.005;
    private final double kV_step = 0.0001;
    private final double kP_step = 0.001;

    private double lastError = 0;
    private int oscillationCount = 0;

    public FlywheelTuningCommand(FlywheelTunerSubsystem flywheel, double maxVel) {
        this.flywheel = flywheel;
        this.maxVel = maxVel;
    }

    @Override
    public void initialize() {
        state = State.ZERO;
    }

    @Override
    public void execute() {
        double velocity = flywheel.getVelocity();
        double error = setpoint - velocity;

        TelemetryData logTelem = new TelemetryData(flywheel.getName(), state, velocity, setpoint, error, kS, kP, kV);
        logTelem.log();

        switch (state) {

            case ZERO:
                kS = kP = kV = 0;
                apply();
                state = State.HIGH_SETPOINT;
                break;

            case HIGH_SETPOINT:
                setpoint = 0.8 * maxVel;
                flywheel.setVelocity(setpoint);
                state = State.FIND_KS_HIGH;
                break;

            case FIND_KS_HIGH:
                if (velocity < 0.05 * setpoint) {
                    kS += kS_step;
                    apply();
                } else {
                    kS -= kS_step;
                    apply();
                    state = State.INIT_KP;
                }
                break;

            case INIT_KP:
                kP = 10.0 / setpoint;
                apply();
                state = State.FIND_KV;
                break;

            case FIND_KV:
                if (Math.abs(error) > 0.05 * setpoint) {
                    kV += kV_step;
                    apply();
                } else {
                    state = State.LOW_SETPOINT;
                }
                break;

            case LOW_SETPOINT:
                setpoint = 0.1 * maxVel;
                flywheel.setVelocity(setpoint);
                state = State.REFINE_KS;
                break;

            case REFINE_KS:
                if (velocity < 0.95 * setpoint) {
                    kS += kS_step;
                    apply();
                } else {
                    state = State.RETURN_HIGH;
                }
                break;

            case RETURN_HIGH:
                setpoint = 0.8 * maxVel;
                flywheel.setVelocity(setpoint);
                state = State.FIND_KP;
                break;

            case FIND_KP:
                if (!isOscillating(error)) {
                    kP += kP_step;
                    apply();
                } else {
                    kP -= kP_step;
                    apply();
                    state = State.VERIFY;
                }
                break;

            case VERIFY:
                // hold multiple setpoints over time (non-blocking)
                flywheel.setVelocity(0.2 * maxVel);
                state = State.DONE;
                break;

            case DONE:
                flywheel.configMotor(kP,kS,kV);
                break;
        }

        lastError = error;
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }

    private void apply() {
        flywheel.setKS(kS);
        flywheel.setKP(kP);
        flywheel.setKV(kV);
    }

    private boolean isOscillating(double error) {
        // Detect zero-crossings of error (VERY effective)
        if (Math.signum(error) != Math.signum(lastError)) {
            oscillationCount++;
        }

        return oscillationCount > 6; // threshold
    }
}