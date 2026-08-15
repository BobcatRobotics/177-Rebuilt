package frc.robot.subsystems.Carwash;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class CarwashIOTalonFXSim implements CarwashIO {

    private final TalonFX motor;

    private final TalonFXSimState motorSimState;

    private final FlywheelSim flywheelSim;

    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);
    public final double SIM_MOI = 0.00003;
    public final double SIM_GEAR_RATIO = 1.0;

    public CarwashIOTalonFXSim(int deviceId) {

        motor = new TalonFX(deviceId);

        motorSimState = motor.getSimState();

        DCMotor motorModel = DCMotor.getKrakenX60Foc(1);

        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        motorModel,
                        this.SIM_MOI,
                        this.SIM_GEAR_RATIO),
                motorModel);
    }

    @Override
    public void setVelocity(double rps) {

        motor.setControl(velocityRequest.withVelocity(rps));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public double getVelocityRPS() {

        return flywheelSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    }

    @Override
    public boolean atTargetSpeed(double targetRPS) {
        return Math.abs(getVelocityRPS() - targetRPS) <= CarwashConstants.SPEED_TOLERANCE_RPS;
    }

    @Override
    public void simulationPeriodic() {

        motorSimState.setSupplyVoltage(12.0);
        double motorVoltage = motorSimState.getMotorVoltage();

        flywheelSim.setInputVoltage(motorVoltage);
        flywheelSim.update(0.020);

        double velocityRPS = flywheelSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

        motorSimState.setRotorVelocity(velocityRPS);
    }
}