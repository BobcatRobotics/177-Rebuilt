package org.bobcatrobotics.Subsystems.FlywheelTuner;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private final ShuffleboardTab tab;

    private final GenericEntry kSEntry;
    private final GenericEntry kPEntry;
    private final GenericEntry kVEntry;

    private final GenericEntry setpointEntry;
    private final GenericEntry velocityEntry;
    private final GenericEntry stateEntry;

    // FOC control request
    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

    // Gains (you dynamically tune these)
    private double kS = 0.0;
    private double kP = 0.0;
    private double kV = 0.0;

    private double setpoint = 0.0;

    public FlywheelSubsystem(int canID) {
        motor = new TalonFX(canID);

        motor.getConfigurator().apply(new TalonFXConfiguration());

        tab = Shuffleboard.getTab("/flywheel/tuning");

        kSEntry = tab.add("kS", 0).getEntry();
        kPEntry = tab.add("kP", 0).getEntry();
        kVEntry = tab.add("kV", 0).getEntry();

        setpointEntry = tab.add("Setpoint", 0).getEntry();
        velocityEntry = tab.add("Velocity", 0).getEntry();
        stateEntry = tab.add("State", "").getEntry();
    }

    public FlywheelSubsystem(TalonFX motor) {
        this.motor = motor;

        tab = Shuffleboard.getTab("/flywheel/tuning");

        kSEntry = tab.add("kS", 0).getEntry();
        kPEntry = tab.add("kP", 0).getEntry();
        kVEntry = tab.add("kV", 0).getEntry();

        setpointEntry = tab.add("Setpoint", 0).getEntry();
        velocityEntry = tab.add("Velocity", 0).getEntry();
        stateEntry = tab.add("State", "").getEntry();
    }

    public void setVelocity(double velocityRPS) {
        setpoint = velocityRPS;

        double ff = kS * Math.signum(velocityRPS) + kV * velocityRPS;

        motor.setControl(
                velocityRequest
                        .withVelocity(velocityRPS)
                        .withFeedForward(ff));
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setKS(double kS) {
        this.kS = kS;
    }

    public void setKP(double kP) {
        this.kP = kP;
    }

    public void setKV(double kV) {
        this.kV = kV;
    }

    public double getKS() {
        return kS;
    }

    public double getKP() {
        return kP;
    }

    public double getKV() {
        return kV;
    }

    public void stop() {
        motor.set(0);
    }

    public void updateDashboard(String state, double setpoint, double velocity) {
    kSEntry.setDouble(kS);
    kPEntry.setDouble(kP);
    kVEntry.setDouble(kV);

    setpointEntry.setDouble(setpoint);
    velocityEntry.setDouble(velocity);
    stateEntry.setString(state);
}
}
