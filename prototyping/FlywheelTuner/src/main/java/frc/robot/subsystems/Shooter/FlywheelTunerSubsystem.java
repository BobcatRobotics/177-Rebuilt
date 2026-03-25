package frc.robot.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FlywheelTunerSubsystem  {

    private final List<TalonFX> mainFlywheelMotors = new ArrayList<>();

    // FOC control request
    private final VelocityTorqueCurrentFOC velocityRequest =
            new VelocityTorqueCurrentFOC(0);

    // Gains (you dynamically tune these)
    private double kS = 0.0;
    private double kP = 0.0;
    private double kV = 0.0;

    private double setpoint = 0.0;

    private String name = "";


    public FlywheelTunerSubsystem(String name,List<Integer> canIDs,List<Boolean> inversions, List<Boolean> coastModes, double supplyCurrent , double statorCurrent, double peakForward, double peakReverse) {
        this.name = name;
        for(int c=0;c<canIDs.size();c++){
            TalonFX motor = new TalonFX(canIDs.get(c));
            TalonFXConfiguration fxConfig = new TalonFXConfiguration();
            motor.getConfigurator().apply(fxConfig);

            fxConfig.MotorOutput.Inverted = inversions.get(c)
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;

            fxConfig.MotorOutput.NeutralMode = coastModes.get(c)
                    ? NeutralModeValue.Coast
                    : NeutralModeValue.Brake;

            fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            fxConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrent;

            fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            fxConfig.CurrentLimits.StatorCurrentLimit = statorCurrent;

            fxConfig.TorqueCurrent.PeakForwardTorqueCurrent = peakForward;
            fxConfig.TorqueCurrent.PeakReverseTorqueCurrent = peakReverse;

            motor.getConfigurator().apply(fxConfig);
            
            mainFlywheelMotors.add(motor);
        }
    }

    public void setVelocity(double velocityRPS) {
        setpoint = velocityRPS;

        double ff = kS * Math.signum(velocityRPS) + kV * velocityRPS;
        for (TalonFX motor : mainFlywheelMotors) {
                    motor.setControl(
            velocityRequest
                .withVelocity(velocityRPS)
                .withFeedForward(ff)
        );
        }

    }

    public double getVelocity() {
        double velocity = 0.0;
        for (TalonFX motor : mainFlywheelMotors) {
                    velocity += motor.getVelocity().getValueAsDouble();
        }
        return velocity/mainFlywheelMotors.size();
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setKS(double kS) { this.kS = kS; }
    public void setKP(double kP) { this.kP = kP; }
    public void setKV(double kV) { this.kV = kV; }

    public double getKS() { return kS; }
    public double getKP() { return kP; }
    public double getKV() { return kV; }

    public void stop() {
        for (TalonFX motor : mainFlywheelMotors) {
             motor.setControl(velocityRequest
                .withVelocity(0));
            motor.stopMotor();
        }
    }

    public String getName(){
        return name;
    }


    public void configMotor(double kP, double kS, double kV){
        for (TalonFX motor : mainFlywheelMotors) {
            Slot0Configs pidConfig = new Slot0Configs();
            pidConfig.kP = kP;
            pidConfig.kS = kS;
            pidConfig.kV = kV;
            motor.getConfigurator().apply(pidConfig);
        }
    }
}
