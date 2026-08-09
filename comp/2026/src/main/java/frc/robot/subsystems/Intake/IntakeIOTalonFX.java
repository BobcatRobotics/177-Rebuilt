package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {

    private TalonFX motor;
    private VelocityVoltage velocityRequest = new VelocityVoltage(0);

    IntakeIOTalonFX(TalonFX motor){
        this.motor = motor;
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
        return motor.getVelocity().getValueAsDouble();
    }
    
}
