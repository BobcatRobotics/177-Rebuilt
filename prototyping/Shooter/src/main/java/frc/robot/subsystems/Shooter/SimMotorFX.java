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
    public enum MotorType {
        FALCON,
        FALCON_FOC,
        KRAKEN,
        KRAKEN_FOC
    }
    private final DCMotor motorModel;
    private final DCMotorSim sim;
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
        motorModel = DCMotor.getKrakenX60Foc(1);
        sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, .025, 1),
                motorModel);
        setTorque(0);
    }

    public SimMotorFX(MotorType motorType) {
        motorModel = selectMotor(motorType);

        sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, .025, 1),
                motorModel);
        setTorque(0);
    }

    private DCMotor selectMotor(MotorType type) {
        return switch (type) {
            case FALCON      -> DCMotor.getFalcon500(1);
            case FALCON_FOC  -> DCMotor.getFalcon500Foc(1);
            case KRAKEN      -> DCMotor.getKrakenX60(1);
            case KRAKEN_FOC  -> DCMotor.getKrakenX60Foc(1);
        };
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
