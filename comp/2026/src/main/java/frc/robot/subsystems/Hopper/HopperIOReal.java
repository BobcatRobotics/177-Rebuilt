package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

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

        config.Slot0.kS = Constants.HopperConstants.Top.kHopperS;
        config.Slot0.kV= Constants.HopperConstants.Top.kHopperV;
        config.Slot0.kP= Constants.HopperConstants.Top.kHopperP;
        config.Slot0.kI= Constants.HopperConstants.Top.kHopperI;
        config.Slot0.kD= Constants.HopperConstants.Top.kHopperD;

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
