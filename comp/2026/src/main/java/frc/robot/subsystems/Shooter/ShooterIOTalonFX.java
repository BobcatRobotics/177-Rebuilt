package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOTalonFX implements ShooterIO {

    private final TalonFX motor;

    private final VelocityVoltage velocityRequest =
            new VelocityVoltage(0);

    public ShooterIOTalonFX(TalonFX motor) {
        this.motor = motor;
    }

    @Override
    public void setVelocity(double rps) {

        motor.setControl(
            velocityRequest.withVelocity(rps)
        );
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public double getVelocityRPS() {

        return motor.getVelocity()
                .getValueAsDouble();
    }
}