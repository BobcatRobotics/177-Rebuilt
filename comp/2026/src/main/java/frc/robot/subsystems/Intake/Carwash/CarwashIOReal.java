package frc.robot.subsystems.Intake.Carwash;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CarwashIOReal implements CarwashIO{
    private TalonFX carwash_Motor;
    private final TalonFXConfigurator carwash_config;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public CarwashIOReal(int id, String bus){
        this.carwash_Motor = new TalonFX(id);
        carwash_config =  carwash_Motor.getConfigurator();
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;

        config.Slot0.kP= 8.0;
        config.Slot0.kV= 7.0;

        carwash_config.apply(config);
    }

    public void periodic(){

    }
    public void setState(){

    }
    
    @Override
    public void stop(){
        carwash_Motor.stopMotor();
    }
    @Override
    public void setRPS(double rps) {
        carwash_Motor.setControl(velocityRequest.withVelocity(rps));
    }
    @Override
    public void simulationPeriodic(){
        
    }
}
