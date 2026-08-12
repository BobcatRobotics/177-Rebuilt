package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;


import frc.robot.subsystems.Hopper.HopperState;

public class IntakeIOReal implements IntakeIO{
    private TalonFX intake_motor;
    private final TalonFXConfigurator intake_config;

    public IntakeIOReal(int id, String bus){
        this.intake_motor = new TalonFX(id,bus);
        intake_config = new TalonFXConfigurator(null);

}
    public void periodic(){

    }
    public void setState(){

    }
    public void stop(){
        intake_motor.stopMotor();
    }
    public void setRPS(HopperState currentState) {
        //set Velocity through a request (currentState.getRPS())
    }
}
