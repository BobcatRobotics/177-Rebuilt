package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class HopperIOTalonFX implements HopperIO {

    private TalonFX motor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0); // should we use HopperState.IDLE.getRPS()?

    //TODO: PID configurations
    public HopperIOTalonFX(TalonFX motor){
        this.motor = motor;
    }

    @Override
    public void setVelocity(double rps){
        motor.setControl(velocityRequest.withVelocity(rps));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public double getVelocityRPS() {
        return motor.getVelocity().getValueAsDouble();
    }
}
