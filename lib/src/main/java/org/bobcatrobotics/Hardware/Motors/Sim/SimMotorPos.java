
package org.bobcatrobotics.Hardware.Motors.Sim;

import edu.wpi.first.math.geometry.Rotation2d;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimMotorPos implements SimMotor {

    private TalonFX motor;
    private double TurnInertia = 0.004;
    private double TurnMotorGearRatio = 1.0;
    private double TURN_KP = 8.0;
    private double TURN_KD = 0.0;
    private DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private final DCMotorSim turnSim;
    private boolean turnClosedLoop = false;
    private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
    private PidContants pidConstants;
    private boolean turnConnected = true;
    private Rotation2d turnPosition = new Rotation2d();
    private double turnVelocityRadPerSec = 0.0;
    private double turnAppliedVolts = 0.0;
    private double turnCurrentAmps = 0.0;

    public SimMotorPos(TalonFX motor, boolean isInverted, PidContants pidConstants, PIDController pidController) {
        this.pidConstants = pidConstants;
        this.turnController = pidController;
        this.motor = motor;
        
        turnSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        TURN_GEARBOX, TurnInertia, TurnMotorGearRatio),
                TURN_GEARBOX);

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    // Update based on a target setpoint
    public void update() {

        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
        } else {
            turnController.reset();
        }

        // Update simulation state
        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        turnSim.update(0.02);
        // Update turn inputs
        turnConnected = true;
        turnPosition = Rotation2d.fromRadians(turnSim.getAngularPositionRad());
        turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());


        // in simulationPeriodic()
        var sim = motor.getSimState();
        // set supply voltage (important!)
        sim.setSupplyVoltage(RobotController.getBatteryVoltage());
        // simulate rotor velocity (rot/s)
        double turnVelocityRotPerSec = turnVelocityRadPerSec / (2 * Math.PI);
        sim.setRotorVelocity(turnVelocityRotPerSec);
        // simulate rotor velocity (rotations)
        double turnPositionRotations = turnPosition.getRotations();
        sim.setRawRotorPosition(turnPositionRotations);
        // simulate voltage
        sim.setSupplyVoltage(turnAppliedVolts);


    }

    public double get() {
        return turnPosition.getRotations();
    }

    public void set(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    public void setPosition(Rotation2d position) {
        turnClosedLoop = true;
        turnController.setSetpoint(position.getRadians());
    }
}
