package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.HopperConstants;

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
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;

        config.Slot0.kS = HopperConstants.kHopperS;
        config.Slot0.kV= HopperConstants.kHopperV;
        config.Slot0.kP= HopperConstants.kHopperP;
        config.Slot0.kI= HopperConstants.kHopperI;
        config.Slot0.kD= HopperConstants.kHopperD;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;

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


