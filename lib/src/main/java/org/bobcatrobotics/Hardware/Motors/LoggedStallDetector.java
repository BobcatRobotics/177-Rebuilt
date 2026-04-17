package org.bobcatrobotics.Hardware.Motors;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
/**
 * 
 * import com.ctre.phoenix6.hardware.TalonFX;

TalonFX motor = new TalonFX(1);

LoggedStallDetector stallDetector = new LoggedStallDetector(
    "IntakeMotor", // unique name per motor
    1.0,           // velocity threshold
    40.0,          // current threshold
    0.25           // time threshold
);

public void updateInputs() {
    double velocity = motor.getVelocity().getValueAsDouble();
    double current  = motor.getStatorCurrent().getValueAsDouble();
    // OR use TORQUE Current if using VelocityTorque Current FOC
    double current  = motor.getTorqueCurrent().getValueAsDouble();

    inputs.stalled = stallDetector.update(velocity, current);
}
 * 
 */
public class LoggedStallDetector {

    private final String name;

    private final double velocityThreshold;
    private final double currentThreshold;
    private final double timeThreshold;

    private double stallStartTime = -1;
    private boolean stalled = false;
    private boolean lastStalled = false;

    public LoggedStallDetector(
            String name,
            double velocityThreshold,
            double currentThreshold,
            double timeThreshold
    ) {
        this.name = name;
        this.velocityThreshold = velocityThreshold;
        this.currentThreshold = currentThreshold;
        this.timeThreshold = timeThreshold;
    }

    /**
     * Update the stall detector and log to AdvantageKit
     */
    public boolean update(double velocity, double current) {
        double now = Timer.getFPGATimestamp();

        boolean candidate =
                Math.abs(velocity) < velocityThreshold &&
                current > currentThreshold;

        if (candidate) {
            if (stallStartTime < 0) {
                stallStartTime = now;
            }

            stalled = (now - stallStartTime) >= timeThreshold;
        } else {
            stallStartTime = -1;
            stalled = false;
        }

        if (stalled && !lastStalled) {
            Logger.recordOutput(name+"/Stall/Event", "STALL_START");
        } else if (!stalled && lastStalled) {
            Logger.recordOutput(name+"/Stall/Event", "STALL_END");
        }

        lastStalled = stalled;

        // 📊 Continuous logging (Elastic + AdvantageScope)
        Logger.recordOutput(name+"/Stall/Velocity", velocity);
        Logger.recordOutput(name+"/Stall/Current", current);
        Logger.recordOutput(name+"/Stall/IsCandidate", candidate);
        Logger.recordOutput(name+"/Stall/IsStalled", stalled);
        Logger.recordOutput(name+"/Stall/TimeSinceStart",
                stallStartTime < 0 ? 0 : now - stallStartTime);

        return stalled;
    }

    public boolean isStalled() {
        return stalled;
    }

    public void reset() {
        stallStartTime = -1;
        stalled = false;
        lastStalled = false;
    }
}