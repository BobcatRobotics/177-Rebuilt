package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOReal implements ShooterIO{
    //Shooter Motors
    private TalonFX dumperLeftUp;
    private TalonFX dumperLeftDown;
    private TalonFX dumperRightUp;
    private TalonFX dumperRightDown;

    //Hood Motor
    private TalonFX adjustableHood;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    //position voltage request for the hood
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public ShooterIOReal(int dumperLeftUpID, int dumperLeftDownID, int dumperRightUpID, int dumperRightDownID, int adjustableHoodID){
        //Shooter Initializations
        this.dumperLeftUp = new TalonFX(dumperLeftUpID);
        this.dumperLeftDown = new TalonFX(dumperLeftDownID);
        this.dumperRightUp = new TalonFX(dumperRightUpID);
        this.dumperRightUp = new TalonFX(dumperRightDownID);
        //Hood Initializations
        this.adjustableHood= new TalonFX(adjustableHoodID);
        //applying configs
        dumperLeftUp.getConfigurator().apply(LeftDumper());
        dumperLeftDown.getConfigurator().apply(LeftDumper());
        dumperRightUp.getConfigurator().apply(RightDumper());
        dumperRightDown.getConfigurator().apply(RightDumper());
        adjustableHood.getConfigurator().apply(adjustableHoodConfig());


    }
    public void periodic(){

    }
    public void stop(){
        dumperLeftUp.stopMotor();
        dumperLeftDown.stopMotor();
        dumperRightUp.stopMotor();
        dumperRightDown.stopMotor();
        adjustableHood.stopMotor();

    }
    @Override
      public void setRPS(double rps, double pos) {
        dumperLeftUp.setControl(velocityRequest.withVelocity(rps));
        dumperLeftDown.setControl(velocityRequest.withVelocity(rps));
        dumperRightUp.setControl(velocityRequest.withVelocity(rps));
        dumperRightDown.setControl(velocityRequest.withVelocity(rps));
        adjustableHood.setControl(positionRequest.withPosition(pos));
    }
    private TalonFXConfiguration LeftDumper(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; //Brake or coast?
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.Slot0.kS = 0;
        config.Slot0.kV = 0;
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kA = 0;
        return config;
    }

    private TalonFXConfiguration RightDumper(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; //Brake or coast?
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.Slot0.kS = 0;
        config.Slot0.kV = 0;
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kA = 0;
        return config;
    }
        private TalonFXConfiguration adjustableHoodConfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.Slot0.kS = 0;
        config.Slot0.kV = 0;
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kA = 0;
        return config;
    }
}
