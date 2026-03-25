package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

public final class TelemetryData {

    private final FlywheelTuningCommand.State state;
    private final double velocity;
    private final double setpoint;
    private final double error;
    private final double kS;
    private final double kP;
    private final double kV;

    private String name;
    public TelemetryData(
        String name,
            FlywheelTuningCommand.State state,
            double velocity,
            double setpoint,
            double error,
            double kS,
            double kP,
            double kV
    ) {
        this.name = name;
        this.state = state;
        this.velocity = velocity;
        this.setpoint = setpoint;
        this.error = error;
        this.kS = kS;
        this.kP = kP;
        this.kV = kV;
    }

    // ✅ Public logging method (no mutation)
    public void log() {
        Logger.recordOutput(name + "/Tuning/State", state.toString());
        Logger.recordOutput(name + "/Tuning/Velocity", velocity);
        Logger.recordOutput(name + "/Tuning/Setpoint", setpoint);
        Logger.recordOutput(name + "/Tuning/Error", error);

        Logger.recordOutput(name + "/Tuning/kS", kS);
        Logger.recordOutput(name + "/Tuning/kP", kP);
        Logger.recordOutput(name + "/Tuning/kV", kV);
    }

    // ✅ (Optional but recommended) getters
    public FlywheelTuningCommand.State getState() { return state; }
    public double getVelocity() { return velocity; }
    public double getSetpoint() { return setpoint; }
    public double getError() { return error; }
    public double getKS() { return kS; }
    public double getKP() { return kP; }
    public double getKV() { return kV; }
}