package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.bobcatrobotics.Util.Tunables.Gains;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Modules.ModuleConfigurator;

public class IntakeReal implements IntakeIO {
    private TalonFX positionMotor;
    public ModuleConfigurator intakePivotConfig;
    private TalonFX leftVelocityMotor;
    public ModuleConfigurator leftintakeVelocityConfig;
    private TalonFX rightVelocityMotor;
    public ModuleConfigurator rightIntakeVelocityConfig;

    // Motor Control Requests
    private TorqueCurrentFOC characterizationRequestTorqueCurrentFOC = new TorqueCurrentFOC(0);
    private VoltageOut characterizationRequestVoltage = new VoltageOut(0);
    private final VelocityVoltage requestVelocity = new VelocityVoltage(0);
    private final PositionTorqueCurrentFOC requestPosition = new PositionTorqueCurrentFOC(0);
    private final PositionVoltage requestPositionVoltage = new PositionVoltage(0);

    // Telemetry Information
    private StatusSignal<AngularVelocity> leftVelocityOfIntakeSpeedRPS;
    private StatusSignal<Current> leftStatorCurrentOfIntakeSpeedAmps;
    private StatusSignal<Voltage> leftOutputOfIntakeSpeedVolts;
    private StatusSignal<AngularAcceleration> leftAccelerationOfIntakeSpeed;

    private StatusSignal<AngularVelocity> rightVelocityOfIntakeSpeedRPS;
    private StatusSignal<Current> rightStatorCurrentOfIntakeSpeedAmps;
    private StatusSignal<Voltage> rightOutputOfIntakeSpeedVolts;
    private StatusSignal<AngularAcceleration> rightAccelerationOfIntakeSpeed;

    private StatusSignal<AngularVelocity> velocityOfIntakePositionRPS;
    private StatusSignal<Current> statorCurrentOfIntakePositionAmps;
    private StatusSignal<Voltage> outputOfIntakePositionVolts;
    private StatusSignal<AngularAcceleration> accelerationOfIntakePosition;

    // Gain Configuration
    Gains pivotMotorGains;
    Gains rightRollerMotorGains;
    Gains leftRollerMotorGains;


    public IntakeReal() {

        pivotMotorGains = new Gains.Builder()
                .kP(Constants.IntakeConstants.PivotConstants.kP)
                .kI(Constants.IntakeConstants.PivotConstants.kI)
                .kD(Constants.IntakeConstants.PivotConstants.kD)
                .kS(Constants.IntakeConstants.PivotConstants.kS)
                .kV(Constants.IntakeConstants.PivotConstants.kV)
                .kA(Constants.IntakeConstants.PivotConstants.kA)
                .build();
        rightRollerMotorGains = new Gains.Builder()
                .kP(Constants.IntakeConstants.RightRollerConstants.kP)
                .kI(Constants.IntakeConstants.RightRollerConstants.kI)
                .kD(Constants.IntakeConstants.RightRollerConstants.kD)
                .kS(Constants.IntakeConstants.RightRollerConstants.kS)
                .kV(Constants.IntakeConstants.RightRollerConstants.kV)
                .kA(Constants.IntakeConstants.RightRollerConstants.kA).build();

        leftRollerMotorGains = new Gains.Builder()
                .kP(Constants.IntakeConstants.LeftRollerConstants.kP)
                .kI(Constants.IntakeConstants.LeftRollerConstants.kI)
                .kD(Constants.IntakeConstants.LeftRollerConstants.kD)
                .kS(Constants.IntakeConstants.LeftRollerConstants.kS)
                .kV(Constants.IntakeConstants.LeftRollerConstants.kV)
                .kA(Constants.IntakeConstants.LeftRollerConstants.kA).build();
        setUpLeftRollerMotor(leftRollerMotorGains);
        setUpRightRollerMotor(rightRollerMotorGains);
        setupPivotMotor(pivotMotorGains);


    }

    public void setUpRightRollerMotor(Gains g) {
        rightIntakeVelocityConfig = new ModuleConfigurator(g.toSlot0Configs(),
                Constants.IntakeConstants.RightRollerConstants.rollerMotorId,
                Constants.IntakeConstants.RightRollerConstants.isInverted,
                Constants.IntakeConstants.RightRollerConstants.isCoast,
                Constants.IntakeConstants.RightRollerConstants.currentLimit,
                Constants.IntakeConstants.RightRollerConstants.peakForwardLimit,
                Constants.IntakeConstants.RightRollerConstants.peakReverseLimit);
        rightVelocityMotor = new TalonFX(rightIntakeVelocityConfig.getMotorId(), new CANBus("rio"));
        rightIntakeVelocityConfig.configureMotor(rightVelocityMotor, g);
        if (Constants.lowTelemetryMode) {
            rightVelocityOfIntakeSpeedRPS = rightVelocityMotor.getVelocity();
            rightStatorCurrentOfIntakeSpeedAmps = rightVelocityMotor.getStatorCurrent();
            rightOutputOfIntakeSpeedVolts = rightVelocityMotor.getMotorVoltage();
            rightIntakeVelocityConfig.configureSignals(rightVelocityMotor, 50.0, rightVelocityOfIntakeSpeedRPS,
                    rightStatorCurrentOfIntakeSpeedAmps, rightOutputOfIntakeSpeedVolts);
        } else {
            rightVelocityOfIntakeSpeedRPS = rightVelocityMotor.getVelocity();
            rightStatorCurrentOfIntakeSpeedAmps = rightVelocityMotor.getStatorCurrent();
            rightOutputOfIntakeSpeedVolts = rightVelocityMotor.getMotorVoltage();
            rightAccelerationOfIntakeSpeed = rightVelocityMotor.getAcceleration();
            rightIntakeVelocityConfig.configureSignals(rightVelocityMotor, 50.0, rightVelocityOfIntakeSpeedRPS,
                    rightStatorCurrentOfIntakeSpeedAmps, rightOutputOfIntakeSpeedVolts, rightAccelerationOfIntakeSpeed);
        }

    }

    public void setUpLeftRollerMotor(Gains g) {
        leftintakeVelocityConfig = new ModuleConfigurator(g.toSlot0Configs(),
                Constants.IntakeConstants.LeftRollerConstants.rollerMotorId,
                Constants.IntakeConstants.LeftRollerConstants.isInverted,
                Constants.IntakeConstants.LeftRollerConstants.isCoast,
                Constants.IntakeConstants.LeftRollerConstants.currentLimit,
                Constants.IntakeConstants.LeftRollerConstants.peakForwardLimit,
                Constants.IntakeConstants.LeftRollerConstants.peakReverseLimit);
        leftVelocityMotor = new TalonFX(leftintakeVelocityConfig.getMotorId(), new CANBus("rio"));
        leftintakeVelocityConfig.configureMotor(leftVelocityMotor, g);
        if (Constants.lowTelemetryMode) {
            leftVelocityOfIntakeSpeedRPS = leftVelocityMotor.getVelocity();
            leftStatorCurrentOfIntakeSpeedAmps = leftVelocityMotor.getStatorCurrent();
            leftOutputOfIntakeSpeedVolts = leftVelocityMotor.getMotorVoltage();
            leftintakeVelocityConfig.configureSignals(leftVelocityMotor, 50.0, leftVelocityOfIntakeSpeedRPS,
                    leftStatorCurrentOfIntakeSpeedAmps, leftOutputOfIntakeSpeedVolts);
        } else {
            leftVelocityOfIntakeSpeedRPS = leftVelocityMotor.getVelocity();
            leftStatorCurrentOfIntakeSpeedAmps = leftVelocityMotor.getStatorCurrent();
            leftOutputOfIntakeSpeedVolts = leftVelocityMotor.getMotorVoltage();
            leftAccelerationOfIntakeSpeed = leftVelocityMotor.getAcceleration();
            leftintakeVelocityConfig.configureSignals(leftVelocityMotor, 50.0, leftVelocityOfIntakeSpeedRPS,
                    leftStatorCurrentOfIntakeSpeedAmps, leftOutputOfIntakeSpeedVolts, leftAccelerationOfIntakeSpeed);
        }
    }

    public void setupPivotMotor(Gains g) {
        intakePivotConfig = new ModuleConfigurator(g.toSlot0Configs(),
                Constants.IntakeConstants.PivotConstants.pivotMotorId,
                Constants.IntakeConstants.PivotConstants.isInverted,
                Constants.IntakeConstants.PivotConstants.isCoast,
                Constants.IntakeConstants.PivotConstants.currentLimit,
                Constants.IntakeConstants.PivotConstants.peakForwardLimit,
                Constants.IntakeConstants.PivotConstants.peakReverseLimit);
        positionMotor = new TalonFX(intakePivotConfig.getMotorId(), new CANBus("rio"));
        intakePivotConfig.configureMotor(positionMotor, g);
        if (Constants.lowTelemetryMode) {
            velocityOfIntakePositionRPS = positionMotor.getVelocity();
            statorCurrentOfIntakePositionAmps = positionMotor.getStatorCurrent();
            outputOfIntakePositionVolts = positionMotor.getMotorVoltage();
            intakePivotConfig.configureSignals(positionMotor, 50.0, velocityOfIntakePositionRPS,
                    statorCurrentOfIntakePositionAmps, outputOfIntakePositionVolts);
        } else {
            velocityOfIntakePositionRPS = positionMotor.getVelocity();
            statorCurrentOfIntakePositionAmps = positionMotor.getStatorCurrent();
            outputOfIntakePositionVolts = positionMotor.getMotorVoltage();
            accelerationOfIntakePosition = positionMotor.getAcceleration();
            intakePivotConfig.configureSignals(positionMotor, 50.0, velocityOfIntakePositionRPS,
                    statorCurrentOfIntakePositionAmps, outputOfIntakePositionVolts, accelerationOfIntakePosition);
        }
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        if (Constants.lowTelemetryMode) {
            lowTelemetry(inputs);
        } else {
            highTelemetry(inputs);
        }

    }

    public void lowTelemetry(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                velocityOfIntakePositionRPS, statorCurrentOfIntakePositionAmps,
                rightVelocityOfIntakeSpeedRPS, rightStatorCurrentOfIntakeSpeedAmps,
                leftVelocityOfIntakeSpeedRPS, leftStatorCurrentOfIntakeSpeedAmps);

        // Position
        inputs.velocityOfIntakePositionRPS = velocityOfIntakePositionRPS.getValue().in(Rotation.per(Minute));
        inputs.statorCurrentOfIntakePositionAmps = statorCurrentOfIntakePositionAmps.getValue().in(Amps);
        inputs.positionConnected = positionMotor.isConnected();
        inputs.intakePosition = positionMotor.getPosition().getValueAsDouble();

        // Right Velocity
        inputs.rightVelocityMotorConnected = rightVelocityMotor.isConnected();
        inputs.rightVelocityOfIntakeSpeedRPS = rightVelocityOfIntakeSpeedRPS.getValue().in(Rotation.per(Minute));
        inputs.rightStatorCurrentOfIntakeSpeedAmps = rightStatorCurrentOfIntakeSpeedAmps.getValue().in(Amps);

        // Left Velocity
        inputs.leftVelocityMotorConnected = leftVelocityMotor.isConnected();
        inputs.leftVelocityOfIntakeSpeedRPS = leftVelocityOfIntakeSpeedRPS.getValue().in(Rotation.per(Minute));
        inputs.leftStatorCurrentOfIntakeSpeedAmps = leftStatorCurrentOfIntakeSpeedAmps.getValue().in(Amps);

    }

    public void highTelemetry(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                accelerationOfIntakePosition, outputOfIntakePositionVolts,
                leftAccelerationOfIntakeSpeed, rightAccelerationOfIntakeSpeed,
                leftOutputOfIntakeSpeedVolts, rightOutputOfIntakeSpeedVolts);

        inputs.accelerationOfIntakePosition = accelerationOfIntakePosition.getValue().in(RotationsPerSecondPerSecond);
        inputs.outputOfIntakePositionVolts = outputOfIntakePositionVolts.getValue().in(Volts);
        inputs.leftAccelerationOfIntakeSpeed = leftAccelerationOfIntakeSpeed.getValue().in(RotationsPerSecondPerSecond);
        inputs.rightAccelerationOfIntakeSpeed = rightAccelerationOfIntakeSpeed.getValue()
                .in(RotationsPerSecondPerSecond);
        inputs.leftOutputOfIntakeSpeedVolts = leftOutputOfIntakeSpeedVolts.getValue().in(Volts);
        inputs.rightOutputOfIntakeSpeedVolts = rightOutputOfIntakeSpeedVolts.getValue().in(Volts);

        lowTelemetry(inputs);
    }

    public void periodic() {

    }

    public void simulationPeriodic() {
    }

    public void runMotors(IntakeState currentState) {
        setVelocity(currentState);
        setPosition(currentState);
    }
    public void setVelocity(IntakeState currentState) {
        rightVelocityMotor.set(currentState.getRollerSpeed());
        leftVelocityMotor.set(currentState.getRollerSpeed());
    }
        public void setPosition(IntakeState currentState) {
        double appliedFeedforward = 0;
        if (currentState == IntakeState.STOW) {
            appliedFeedforward = 0.8;
        } else {
            appliedFeedforward = 0.7;
        }
        positionMotor
                .setControl(requestPositionVoltage.withPosition(currentState.getPosition())
                        .withFeedForward(appliedFeedforward));
    }
    public void stop() {
        positionMotor.stopMotor();
        rightVelocityMotor.stopMotor();
        leftVelocityMotor.stopMotor();
    }


  public boolean atSpeed(IntakeState currentState) {
    boolean isAtTolerance = false;
    boolean isRollerLeftWithinTolerance = false;
    boolean isRollerRightWithinTolerance = false;

    double MAIN_SPEED_TOLERANCE = 5; // Using for both dumpers
    double leftVelocityAvg = (leftVelocityMotor.getVelocity().getValueAsDouble() )/1;
    double rightVelocityAvg =(rightVelocityMotor.getVelocity().getValueAsDouble() )/1;
    isRollerLeftWithinTolerance = Math.abs( leftVelocityAvg
        - currentState.getRollerSpeed()) <= MAIN_SPEED_TOLERANCE;
    isRollerRightWithinTolerance = Math
        .abs(rightVelocityAvg
            - currentState.getRollerSpeed()) <= MAIN_SPEED_TOLERANCE;
    if (isRollerRightWithinTolerance && isRollerLeftWithinTolerance) {
      isAtTolerance = true;
    }
    Logger.recordOutput("Intake/isUpToSpeed", isAtTolerance);
    return isAtTolerance;
  }

  public boolean atPosition(IntakeState currentState) {
    boolean isAtTolerance = false;
    boolean isIntakeWithinTolerance = false;

    double HOOD_POSITION_TOLERANCE = 0.5;
    double hoodpPosition = positionMotor.getPosition().getValueAsDouble();
    isIntakeWithinTolerance = Math
        .abs(hoodpPosition
            - currentState.getPosition()) <= HOOD_POSITION_TOLERANCE;
    if (isIntakeWithinTolerance) {
      isAtTolerance = true;
    }
    Logger.recordOutput("Intake/isAtPosition", isAtTolerance);
    return isAtTolerance;
  }
}
