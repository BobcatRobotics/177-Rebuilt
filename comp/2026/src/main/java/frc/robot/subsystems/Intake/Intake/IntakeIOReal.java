package frc.robot.subsystems.Intake.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;

public class IntakeIOReal implements IntakeIO{
    private final TalonFX intake_right;
    private final TalonFX intake_left;
    private final TalonFXConfigurator intake_config;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public IntakeIOReal(int RightID, int LeftID){
        this.intake_right = new TalonFX(RightID);
        this.intake_left = new TalonFX(LeftID);
        intake_config = intake_right.getConfigurator();
        //intake_left.setControl(new Follower(1, MotorAlignmentValue.Opposed));

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
        intake_right.stopMotor();
    }
    @Override
    public void setRPS(double rps) {
        intake_right.setControl(velocityRequest.withVelocity(rps));
    }
    @Override
    public void simulationPeriodic(){
        
    }
}
