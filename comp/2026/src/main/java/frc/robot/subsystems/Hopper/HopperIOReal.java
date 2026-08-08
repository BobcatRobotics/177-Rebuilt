package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;



public class HopperIOReal implements HopperIO{

    private TalonFX hopper;
    private final TalonFXConfigurator hopper_config;

    public HopperIOReal(int id, String bus){
        this.hopper = new TalonFX(id,bus);

        hopper_config = new TalonFXConfigurator(null);
    }
    public void periodic(){

    }
    public void setState(){

    }
    public void stop(){

    }

}


