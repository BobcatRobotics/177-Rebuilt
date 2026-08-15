package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOTalonFX implements ShooterIO {

    private final TalonFX innerLeft;
    private final TalonFX innerRight;
    private final TalonFX outerLeft;
    private final TalonFX outerRight;

    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

    public ShooterIOTalonFX(int innerLeftId, int innerRightId, int outerLeftId,int outerRightId) {
        innerLeft = new TalonFX(innerLeftId);
        innerRight = new TalonFX(innerRightId);
        outerLeft = new TalonFX(outerLeftId);
        outerRight = new TalonFX(outerRightId);

        innerLeft.getConfigurator().apply(createLeftBaseConfiguration());
        innerRight.getConfigurator().apply(createRightBaseConfiguration());
        outerLeft.getConfigurator().apply(createLeftBaseConfiguration());
        outerRight.getConfigurator().apply(createRightBaseConfiguration());
    }

    @Override
    public void setFlywheelVelocity(double rps) {
        innerLeft.setControl(velocityRequest.withVelocity(rps));
        innerRight.setControl(velocityRequest.withVelocity(rps));
        outerLeft.setControl(velocityRequest.withVelocity(rps));
        outerRight.setControl(velocityRequest.withVelocity(rps));
    }

    @Override
    public void stopFlywheel() {
        innerLeft.stopMotor();
        innerRight.stopMotor();
        outerLeft.stopMotor();
        outerRight.stopMotor();
    }

    @Override
    public double getLeftInnerVelocityRPS() {
        return innerLeft.getVelocity().getValueAsDouble();
    }

    @Override
    public double getRightInnerVelocityRPS() {
        return innerRight.getVelocity().getValueAsDouble();
    }

    @Override
    public double getLeftOuterVelocityRPS() {
        return outerLeft.getVelocity().getValueAsDouble();
    }

    @Override
    public double getRightOuterVelocityRPS() {
        return outerRight.getVelocity().getValueAsDouble();
    }

    @Override
    public double getFlywheelVelocityRPS() {

        return (getLeftInnerVelocityRPS() + getRightInnerVelocityRPS()
                + getLeftOuterVelocityRPS() + getRightOuterVelocityRPS()
                ) / 4.0;
    }

    @Override
    public boolean atTargetSpeed(double targetRPS) {

        return Math.abs(
                getFlywheelVelocityRPS() - targetRPS) < ShooterConstants.SPEED_TOLERANCE_RPS;
    }

    // These are common, in case outer and inner on Left need different settings then create separate methods
    private TalonFXConfiguration createLeftBaseConfiguration() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.Left.supplyCurrentLimit;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.Left.statorCurrentLimit;

        config.Slot0.kS = ShooterConstants.Left.kdumperLeftMotorkS;
        config.Slot0.kV = ShooterConstants.Left.kdumperLeftMotorkV;
        config.Slot0.kP = ShooterConstants.Left.kdumperLeftMotorkP;
        config.Slot0.kI = ShooterConstants.Left.kdumperLeftMotorkI;
        config.Slot0.kD = ShooterConstants.Left.kdumperLeftMotorkD;

        return config;
    }

    private TalonFXConfiguration createRightBaseConfiguration() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.Right.supplyCurrentLimit;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.Right.statorCurrentLimit;

        config.Slot0.kS = ShooterConstants.Right.kdumperRightMotorkS;
        config.Slot0.kV = ShooterConstants.Right.kdumperRightMotorkV;
        config.Slot0.kP = ShooterConstants.Right.kdumperRightMotorkP;
        config.Slot0.kI = ShooterConstants.Right.kdumperRightMotorkI;
        config.Slot0.kD = ShooterConstants.Right.kdumperRightMotorkD;

        return config;
    }
}