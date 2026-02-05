package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// --- Simulated Motor Class ---
public class SimMotorFX {
    private static final DCMotor motorModel = DCMotor.getKrakenX60(1);
    private static final DCMotorSim sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, .025, 1),
            motorModel);
    private double appliedVolts = 0.0;
    private double velocity = 0.0;
    private double acceleration = 0.0;
    private double position = 0.0;
    private double supplyCurrent = 0.0;

    private double appliedTorque = 0.0;

    private double falconStallTorqueInNm = 4.69;
    private double krakenStallTorqueInNm = 7.16;
    private double falconFreeSpeedInRPM = 6380;
    private double krakenFreeSpeedInRPM = 6050;

    public SimMotorFX() {
        setTorque(0);
    }

    // Update based on a target setpoint
    public void update() {
        appliedVolts = motorModel.getVoltage(appliedTorque,
                Units.RotationsPerSecond.convertFrom(sim.getAngularVelocityRadPerSec(), RadiansPerSecond));

        // Update sim state
        sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
        sim.update(0.02);

        supplyCurrent = sim.getCurrentDrawAmps();
        position = Units.Rotations.convertFrom(sim.getAngularPositionRad(), Radians);
        velocity = Units.RotationsPerSecond.convertFrom(sim.getAngularVelocityRadPerSec(), RadiansPerSecond);
        acceleration = Units.RotationsPerSecondPerSecond.convertFrom(sim.getAngularAccelerationRadPerSecSq(),
                RadiansPerSecondPerSecond);
    }

    public double getVelocity() {
        return velocity;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public double getVoltage() {
        return appliedVolts;
    }

    public double getCurrent() {
        return supplyCurrent;
    }

    public double getPosition() {
        return position;
    }

    public double setTorque(double velocityInRPS) {
        double velocityInRMP = velocityInRPS * 60;
        appliedTorque = falconStallTorqueInNm * (1 - (velocityInRMP / falconFreeSpeedInRPM));
        return appliedTorque;
    }
}
