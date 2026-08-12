package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class IntakeIOReal implements IntakeIO{
    private final TalonFX intake_motor;
    private final TalonFXConfigurator intake_config;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public IntakeIOReal(int id, String bus){
        this.intake_motor = new TalonFX(id,bus);
        intake_config = intake_motor.getConfigurator();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intake_config.apply(config);


}
    public void periodic(){

    }
    public void setState(){

    }
    @Override
    public void stop(){
        intake_motor.stopMotor();
    }
    @Override
    public void setRPS(double rps) {
        intake_motor.setControl(velocityRequest.withVelocity(rps));
    }
}
