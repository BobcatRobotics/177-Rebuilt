package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HopperIOReal implements HopperIO{

    private final TalonFX hopper;
    private final TalonFXConfigurator hopperConfig;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public HopperIOReal(int id, String bus){
        this.hopper = new TalonFX(id,bus);
        hopperConfig = hopper.getConfigurator();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        hopperConfig.apply(config);
    }
    public void periodic(){

    }

    public void stop(){
        hopper.stopMotor();
    }
    @Override
      public void setRPS(double rps) {
        hopper.setControl(velocityRequest.withVelocity(rps));
    }

}


