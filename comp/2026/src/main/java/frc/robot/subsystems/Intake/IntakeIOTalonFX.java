package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeIOTalonFX implements IntakeIO {

    private final TalonFX leftRollerMotor;
    private final TalonFX rightRollerMotor;
    private final TalonFX pivotMotor;

    private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0);
    private final PositionVoltage pivotPositionRequest = new PositionVoltage(0);

    public IntakeIOTalonFX(int leftRollerDeviceId, int rightRollerDeviceId, int pivotDeviceId) {
        leftRollerMotor = new TalonFX(leftRollerDeviceId);
        rightRollerMotor = new TalonFX(rightRollerDeviceId);
        pivotMotor = new TalonFX(pivotDeviceId);

        leftRollerMotor.getConfigurator().apply(createLeftRollerConfiguration());
        rightRollerMotor.getConfigurator().apply(createRightRollerConfiguration());
        pivotMotor.getConfigurator().apply(createPivotConfiguration());
    }

    // ROLLERS
    @Override
    public void setRollerVelocity(double rps) {

        leftRollerMotor.setControl(
                rollerVelocityRequest.withVelocity(rps));

        rightRollerMotor.setControl(
                rollerVelocityRequest.withVelocity(rps));
    }

    @Override
    public void stopRollers() {

        leftRollerMotor.stopMotor();
        rightRollerMotor.stopMotor();
    }

    @Override
    public double getLeftRollerVelocityRPS() {

        return leftRollerMotor
                .getVelocity()
                .getValueAsDouble();
    }

    @Override
    public double getRightRollerVelocityRPS() {

        return rightRollerMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getRollerAverageVelocityRPS() {
        double left = getLeftRollerVelocityRPS();
        double right = getRightRollerVelocityRPS();
        return (left + right) / 2.0;
    }

    // PIVOT
    @Override
    public void setPivotAngle(double angleDegrees) {
        pivotMotor.setControl(pivotPositionRequest.withPosition(angleDegrees / 360.0));
    }

    @Override
    public void stopPivot() {
        pivotMotor.stopMotor();
    }

    @Override
    public double getPivotAngleDegrees() {
        return pivotMotor.getPosition().getValueAsDouble() * 360.0;
    }

    @Override
    public boolean isPivotAtTarget(
            double targetAngleDegrees) {

        double currentAngle = getPivotAngleDegrees();

        return Math.abs(
                currentAngle - targetAngleDegrees) < IntakeConstants.PivotConstants.ANGLE_TOLERANCE_DEGREES;
    }

    // CONFIGURATION
    private TalonFXConfiguration createLeftRollerConfiguration() {

        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.MotorOutput.Inverted = IntakeConstants.LeftRollerConstants.isInverted
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;

        configuration.MotorOutput.NeutralMode = IntakeConstants.LeftRollerConstants.isCoast
                ? NeutralModeValue.Coast
                : NeutralModeValue.Brake;

        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.SupplyCurrentLimit = IntakeConstants.LeftRollerConstants.SUPPLY_CURRENT_LIMIT;
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimit = IntakeConstants.LeftRollerConstants.STATOR_CURRENT_LIMIT;

        configuration.Slot0.kS = IntakeConstants.LeftRollerConstants.kS;
        configuration.Slot0.kV = IntakeConstants.LeftRollerConstants.kV;
        configuration.Slot0.kP = IntakeConstants.LeftRollerConstants.kP;
        configuration.Slot0.kI = IntakeConstants.LeftRollerConstants.kI;
        configuration.Slot0.kD = IntakeConstants.LeftRollerConstants.kD;
        return configuration;
    }

    private TalonFXConfiguration createRightRollerConfiguration() {

        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.MotorOutput.Inverted = IntakeConstants.RightRollerConstants.isInverted
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;

        configuration.MotorOutput.NeutralMode = IntakeConstants.RightRollerConstants.isCoast
                ? NeutralModeValue.Coast
                : NeutralModeValue.Brake;

        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.SupplyCurrentLimit = IntakeConstants.RightRollerConstants.SUPPLY_CURRENT_LIMIT;

        configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimit = IntakeConstants.RightRollerConstants.STATOR_CURRENT_LIMIT;

        configuration.Slot0.kS = IntakeConstants.RightRollerConstants.kS;
        configuration.Slot0.kV = IntakeConstants.RightRollerConstants.kV;
        configuration.Slot0.kP = IntakeConstants.RightRollerConstants.kP;
        configuration.Slot0.kI = IntakeConstants.RightRollerConstants.kI;
        configuration.Slot0.kD = IntakeConstants.RightRollerConstants.kD;

        return configuration;
    }

    private TalonFXConfiguration createPivotConfiguration() {

        TalonFXConfiguration configuration = new TalonFXConfiguration();

        configuration.MotorOutput.Inverted = IntakeConstants.PivotConstants.isInverted
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;

        configuration.MotorOutput.NeutralMode = IntakeConstants.PivotConstants.isCoast
                ? NeutralModeValue.Coast
                : NeutralModeValue.Brake;

        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.SupplyCurrentLimit = IntakeConstants.PivotConstants.SUPPLY_CURRENT_LIMIT;

        configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimit = IntakeConstants.PivotConstants.STATOR_CURRENT_LIMIT;

        configuration.Slot0.kS = IntakeConstants.PivotConstants.kS;
        configuration.Slot0.kV = IntakeConstants.PivotConstants.kV;
        configuration.Slot0.kP = IntakeConstants.PivotConstants.kP;
        configuration.Slot0.kI = IntakeConstants.PivotConstants.kI;
        configuration.Slot0.kD = IntakeConstants.PivotConstants.kD;

        return configuration;
    }
}