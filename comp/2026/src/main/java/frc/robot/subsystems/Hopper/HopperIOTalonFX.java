package frc.robot.subsystems.Hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HopperIOTalonFX implements HopperIO {

    private final TalonFX motor;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0); 

    public HopperIOTalonFX(int deviceId){
        this.motor = new TalonFX(deviceId);
        this.motor.getConfigurator().apply(createConfiguration());
    }

    @Override
    public void setVelocity(double rps){
        motor.setControl(velocityRequest.withVelocity(rps));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public double getVelocityRPS() {
        return motor.getVelocity().getValueAsDouble();
    }

    private TalonFXConfiguration createConfiguration(){
        TalonFXConfiguration configuration = new TalonFXConfiguration();
        
        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        configuration.MotorOutput.NeutralMode = HopperConstants.ISCOAST 
            ? NeutralModeValue.Coast 
            : NeutralModeValue.Brake;

        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.SupplyCurrentLimit = HopperConstants.SUPPLY_CURRENT_LIMIT;
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimit = HopperConstants.STATOR_CURRENT_LIMIT;

        configuration.Slot0.kS = HopperConstants.KS;
        configuration.Slot0.kV= HopperConstants.KV;
        configuration.Slot0.kP= HopperConstants.KP;
        configuration.Slot0.kI= HopperConstants.KI;
        configuration.Slot0.kD= HopperConstants.KD;

        return configuration;
    }
}
