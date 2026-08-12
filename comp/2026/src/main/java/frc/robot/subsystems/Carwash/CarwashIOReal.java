package frc.robot.subsystems.Carwash;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.Hopper.HopperState;

public class CarwashIOReal implements CarwashIO{

    private TalonFX carwash_Motor;
    private final TalonFXConfigurator carwash_config;

    public CarwashIOReal(int id, String bus){
        this.carwash_Motor = new TalonFX(id);
        carwash_config = new TalonFXConfigurator(null);

        //Motor Configuration Requirements
    }

    public void periodic(){

    }
    public void setState(){

    }
    public void stop(){
        carwash_Motor.stopMotor();
    }
    public void setRPS(HopperState currentState) {
        //set Velocity through a request (currentState.getRPS())
    }
}
