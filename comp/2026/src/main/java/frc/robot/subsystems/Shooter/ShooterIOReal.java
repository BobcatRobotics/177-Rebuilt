package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOReal implements ShooterIO{
    private final TalonFX shooter;
    //private final TalonFX bottomRight;
    //private final TalonFX topLeft;
    //private final TalonFX bottomLeft;

    private final TalonFXConfigurator shooterConfig;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public ShooterIOReal(int id, String bus){
        this.shooter = new TalonFX(id,bus);
        shooterConfig = shooter.getConfigurator();

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;

        shooterConfig.apply(config);
    }
    public void periodic(){

    }

    public void stop(){
        shooter.stopMotor();
    }
    @Override
      public void setRPS(double rps) {
        shooter.setControl(velocityRequest.withVelocity(rps));
    }
}
