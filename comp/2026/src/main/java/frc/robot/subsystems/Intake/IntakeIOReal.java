package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.HopperConstants;


public class IntakeIOReal implements IntakeIO{
    private final TalonFX intake_motor;
    private final TalonFXConfigurator intake_config;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public IntakeIOReal(int id, String bus){
        this.intake_motor = new TalonFX(id,bus);
        intake_config = intake_motor.getConfigurator();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;


        config.Slot0.kV= 0.11;
        config.Slot0.kP= 1;


        
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
    @Override
    public void simulationPeriodic(){
        
    }
}
