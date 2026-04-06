package frc.robot.subsystems.Intake;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

// --- Simulated Motor Class ---
public class SimMotorFX {
    public enum MotorType {
        FALCON,
        FALCON_FOC,
        KRAKEN,
        KRAKEN_FOC
    }

    private double falconStallTorqueInNm = 4.69;
    private double krakenStallTorqueInNm = 7.16;
    private double falconFreeSpeedInRPM = 6380;
    private double krakenFreeSpeedInRPM = 6050;

    private double velocity = 0.0;
    private double position = 0.0;
    private double voltage = 0.0;

    private TalonFX motor;

    public SimMotorFX(TalonFX motor, boolean isInverted) {
        this.motor = motor;
        // in simulationPeriodic()
        var sim = motor.getSimState();
        sim.Orientation = ChassisReference.CounterClockwise_Positive;
        if(isInverted){
            sim.Orientation = ChassisReference.Clockwise_Positive;
        }
    }

    // Update based on a target setpoint
    public void update() {
        // in simulationPeriodic()
        var sim = motor.getSimState();
        // set supply voltage (important!)
        sim.setSupplyVoltage(RobotController.getBatteryVoltage());
        // simulate rotor velocity (rot/s)
        sim.setRotorVelocity(velocity);
        // simulate rotor velocity (rotations)
        sim.setRawRotorPosition(position);

        sim.setSupplyVoltage(voltage);
    }

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velInRotationsPerSec) {
        this.velocity = velInRotationsPerSec ;
    }

    public void setPosition(double posInRotations){
        this.position = posInRotations;
    }

    public void set(double output){
        this.voltage = RobotController.getBatteryVoltage() * output;
    }
}
