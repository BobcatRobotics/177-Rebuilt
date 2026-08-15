package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HopperIOTalonFXSim implements HopperIO {

    private final TalonFX motor;
    private final TalonFXSimState motorSimState;
    private final DCMotorSim motorSim;
    private final VelocityVoltage velocityRequest =new VelocityVoltage(0);
    
    public final double MOI = 0.001;
    public final double GEAR_RATIO = 1.0;

    public HopperIOTalonFXSim(int deviceId) {

        motor = new TalonFX(deviceId);
        motorSimState = motor.getSimState();
        DCMotor motorModel = DCMotor.getFalcon500(1);

        motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, this.MOI,this.GEAR_RATIO), motorModel);
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
        return motorSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    }


    @Override
    public void simulationPeriodic() {

        double motorVoltage = motorSimState.getMotorVoltage();
        motorSim.setInputVoltage(motorVoltage);
        motorSim.update(0.020);
        motorSimState.setRawRotorPosition(motorSim.getAngularPositionRotations());

        motorSimState.setRotorVelocity(motorSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI));
    }
}
