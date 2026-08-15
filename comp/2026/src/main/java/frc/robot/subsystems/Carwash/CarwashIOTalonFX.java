package frc.robot.subsystems.Carwash;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CarwashIOTalonFX implements CarwashIO {

    private final TalonFX motor;

    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

    public CarwashIOTalonFX(int deviceId) {
        motor = new TalonFX(deviceId);
        motor.getConfigurator().apply(createConfiguration());
    }

    @Override
    public void setVelocity(double rps) {

        motor.setControl(
                velocityRequest.withVelocity(rps));
    }

    @Override
    public void stop() {

        motor.stopMotor();
    }

    @Override
    public double getVelocityRPS() {

        return motor.getVelocity().getValueAsDouble();
    }

    @Override
    public boolean atTargetSpeed(double targetRPS) {

        return Math.abs(
                getVelocityRPS() - targetRPS) <= CarwashConstants.SPEED_TOLERANCE_RPS;
    }

    private TalonFXConfiguration createConfiguration() {

        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.MotorOutput.Inverted = CarwashConstants.isInverted
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;

        configuration.MotorOutput.NeutralMode = CarwashConstants.isCoast
                ? NeutralModeValue.Coast
                : NeutralModeValue.Brake;

        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.SupplyCurrentLimit = CarwashConstants.supplyCurrentLimit;

        configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimit = CarwashConstants.statorCurrentLimit;

        configuration.Slot0.kP = CarwashConstants.kP;
        configuration.Slot0.kI = CarwashConstants.kI;
        configuration.Slot0.kD = CarwashConstants.kD;
        configuration.Slot0.kS = CarwashConstants.kS;
        configuration.Slot0.kV = CarwashConstants.kV;
        configuration.Slot0.kA = CarwashConstants.kA;

        return configuration;
    }
}