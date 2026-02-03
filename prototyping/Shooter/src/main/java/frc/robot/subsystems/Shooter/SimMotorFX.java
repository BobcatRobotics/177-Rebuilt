package frc.robot.subsystems.Shooter;

// --- Simulated Motor Class ---
public class SimMotorFX {
    private double velocity;
    private double lastVelocity;
    private double acceleration;
    private double voltage;
    private double current;    

    // Update based on a target setpoint
    public void update(double setpoint) {
        double dt = 0.02; // 20ms loop
        double maxAccel = 50; // max RPS^2 for simulation
        acceleration = clamp((setpoint - velocity) / dt, -maxAccel, maxAccel);
        lastVelocity = velocity;
        velocity += acceleration * dt;

        // Simple linear approximation for voltage/current
        voltage = setpoint * 12.0; // scale velocity to volts
        current = Math.abs(velocity) * 10; // fake current proportional to speed
    }
    
    // Helper method to clamp a value between min and max
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void setVoltage(double volts) {
        voltage = volts;
        velocity = volts / 12.0; // simple linear scaling
    }

    public double getVelocity() {
        return velocity;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public double getVoltage() {
        return voltage;
    }

    public double getCurrent() {
        return current;
    }
}
