
package org.bobcatrobotics.Hardware.Motors.Sim;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimMotorVel implements SimMotor {
    private double falconStallTorqueInNm = 4.69;
    private double krakenStallTorqueInNm = 7.16;
    private double falconFreeSpeedInRPM = 6380;
    private double krakenFreeSpeedInRPM = 6050;

    private TalonFX motor;

    // Drive "velocity" motors
    private double DriveInertia = 0.025;
    private double DriveMotorGearRatio = 1.0;
    private DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private DCMotorSim driveSim;
    private PIDController pidController;
    private PidContants pidConstants;
    private double driveFFVolts = 0.0;
    private boolean driveClosedLoop = false;
    public boolean driveConnected = true;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public SimMotorVel(TalonFX motor, boolean isInverted, PidContants pidConstants, PIDController pidController) {
        this.pidConstants = pidConstants;
        this.pidController = pidController;
        this.motor = motor;

        driveSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DRIVE_GEARBOX, DriveInertia, DriveMotorGearRatio),
                DRIVE_GEARBOX);

    }

    // Update based on a target setpoint
    public void update() {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts =  pidController.calculate(driveSim.getAngularVelocityRadPerSec());
        } else {
            driveAppliedVolts = 0;
            pidController.reset();
        }

        // Update simulation state
        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        driveSim.update(0.02);

        // Update drive inputs
        driveConnected = true;
        drivePositionRad = driveSim.getAngularPositionRad();
        driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
        driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        // in simulationPeriodic()
        var sim = motor.getSimState();
        // set supply voltage (important!)
        sim.setSupplyVoltage(RobotController.getBatteryVoltage());
        // simulate rotor velocity (rot/s)
        double driveVelocityRotPerSec = driveVelocityRadPerSec / (2 * Math.PI);
        sim.setRotorVelocity(driveVelocityRotPerSec);
        // simulate rotor velocity (rotations)
        double drivePositionRotations = drivePositionRad / (2 * Math.PI);
        sim.setRawRotorPosition(drivePositionRotations);
        // simulate voltage
        sim.setSupplyVoltage(driveAppliedVolts);


    }

    public double getVelocity() {
        return driveVelocityRadPerSec;
    }

    public void set(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    public void setVelocity(double velInRotationsPerSec) {
        driveClosedLoop = true;
        double velocityRadPerSec = velInRotationsPerSec * 2 * Math.PI;
        //driveFFVolts = pidConstants.getKS() * Math.signum(velocityRadPerSec) + pidConstants.getKV() * velocityRadPerSec;
        pidController.setSetpoint(velocityRadPerSec);
    }
}
