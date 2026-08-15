package frc.robot.subsystems.Hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HoodIOTalonFX implements HoodIO {

    private final TalonFX hoodMotor;

    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public HoodIOTalonFX(int motorId) {
        hoodMotor = new TalonFX(motorId);
        hoodMotor.getConfigurator().apply(createBaseConfiguration());
    }

    @Override
    public void setAngle(double angleDegrees) {
        double motorRotations = degreesToMotorRotations(angleDegrees);
        hoodMotor.setControl(positionRequest.withPosition(motorRotations));
    }

    @Override
    public void stop() {
        hoodMotor.stopMotor();
    }

    @Override
    public double getAngleDegrees() {
        return motorRotationsToDegrees(hoodMotor.getPosition().getValueAsDouble());
    }

    @Override
    public boolean atTargetAngle(
            double targetAngleDegrees) {

        return Math.abs(getAngleDegrees() - targetAngleDegrees) <= HoodConstants.ANGLE_TOLERANCE_DEGREES;
    }

    @Override
    public double getHoodMotorVelocityRPS() {

        return hoodMotor.getVelocity().getValueAsDouble();
    }

    private double degreesToMotorRotations(
            double degrees) {

        double hoodRotations = degrees / 360.0;

        return hoodRotations * HoodConstants.GEAR_RATIO;
    }

    private double motorRotationsToDegrees(
            double motorRotations) {

        double hoodRotations = motorRotations / HoodConstants.GEAR_RATIO;

        return hoodRotations * 360.0;
    }

    private TalonFXConfiguration createBaseConfiguration() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = HoodConstants.isInverted
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;

        config.MotorOutput.NeutralMode = HoodConstants.isCoast
                ? NeutralModeValue.Coast
                : NeutralModeValue.Brake;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = HoodConstants.supplyCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = HoodConstants.statorCurrentLimit;

        config.Slot0.kS = HoodConstants.kS;
        config.Slot0.kV = HoodConstants.kV;
        config.Slot0.kP = HoodConstants.kP;
        config.Slot0.kI = HoodConstants.kI;
        config.Slot0.kD = HoodConstants.kD;

        return config;
    }
}
