package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.bobcatrobotics.Hardware.Characterization.CharacterizationClosedLoopOutputType;
import org.bobcatrobotics.Util.Tunables.Gains;
// import org.bobcatrobotics.Util.Tunables.TunablePID;

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
import frc.robot.subsystems.Intake.Modules.ModuleConfigurator;

public class IntakeReal implements IntakeIO {

  private TalonFX positionMotor;
  public ModuleConfigurator intakePivotConfig;

  private TalonFX velocityMotor;
  public ModuleConfigurator intakeVelocityConfig;

  private TorqueCurrentFOC characterizationRequestTorqueCurrentFOC = new TorqueCurrentFOC(0);
  private VoltageOut characterizationRequestVoltage = new VoltageOut(0);
  private final VelocityVoltage requestVelocity = new VelocityVoltage(0);
  private final PositionTorqueCurrentFOC requestPosition = new PositionTorqueCurrentFOC(0);
  private final PositionVoltage requestPositionVoltage = new PositionVoltage(0);

  // private TunablePID intakePivotPID;
  // private TunablePID intakeVelocityPID;

  public double intakePivotSetpoint = 0;
  public double intakeVelocitySetpoint = 0;

  private StatusSignal<AngularVelocity> velocityOfIntakeSpeedRPS;
  private StatusSignal<Current> statorCurrentOfIntakeSpeedAmps;
  private StatusSignal<Voltage> outputOfIntakeSpeedVolts;
  private StatusSignal<AngularAcceleration> accelerationOfIntakeSpeed;

  private StatusSignal<AngularVelocity> velocityOfIntakePositionRPS;
  private StatusSignal<Current> statorCurrentOfIntakePositionAmps;
  private StatusSignal<Voltage> outputOfIntakePositionVolts;
  private StatusSignal<AngularAcceleration> accelerationOfIntakePosition;

  // private FindLimit seekLowerRange;
Gains pivotMotorGains;
Gains rollerMotorGains;
  public IntakeReal() {
    pivotMotorGains = new Gains.Builder()
        .kP(Constants.IntakeConstants.PivotConstants.kP)
        .kI(Constants.IntakeConstants.PivotConstants.kI)
        .kD(Constants.IntakeConstants.PivotConstants.kD)
        .kS(Constants.IntakeConstants.PivotConstants.kS)
        .kV(Constants.IntakeConstants.PivotConstants.kV)
        .kA(Constants.IntakeConstants.PivotConstants.kA)
        .build();
    rollerMotorGains = new Gains.Builder()
        .kP(Constants.IntakeConstants.RightRollerConstants.kP)
        .kI(Constants.IntakeConstants.RightRollerConstants.kI)
        .kD(Constants.IntakeConstants.RightRollerConstants.kD)
        .kS(Constants.IntakeConstants.RightRollerConstants.kS)
        .kV(Constants.IntakeConstants.RightRollerConstants.kV)
        .kA(Constants.IntakeConstants.RightRollerConstants.kA).build();
    setupRollerMotor(rollerMotorGains);
    setupPivotMotor(pivotMotorGains);

    // seekLowerRange = new FindLimit(false, positionMotor);
  }

  public void setupRollerMotor(Gains g) {
    intakeVelocityConfig = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.IntakeConstants.RightRollerConstants.rollerMotorId,
        Constants.IntakeConstants.RightRollerConstants.isInverted,
        Constants.IntakeConstants.RightRollerConstants.isCoast,
        Constants.IntakeConstants.RightRollerConstants.currentLimit,
        Constants.IntakeConstants.RightRollerConstants.peakForwardLimit,
        Constants.IntakeConstants.RightRollerConstants.peakReverseLimit);
    velocityMotor = new TalonFX(intakeVelocityConfig.getMotorId(), new CANBus("rio"));
    intakeVelocityConfig.configureMotor(velocityMotor, g);
    if (Constants.lowTelemetryMode) {
      velocityOfIntakeSpeedRPS = velocityMotor.getVelocity();
      statorCurrentOfIntakeSpeedAmps = velocityMotor.getStatorCurrent();
      outputOfIntakeSpeedVolts = velocityMotor.getMotorVoltage();
      intakeVelocityConfig.configureSignals(velocityMotor, 50.0, velocityOfIntakeSpeedRPS,
          statorCurrentOfIntakeSpeedAmps, outputOfIntakeSpeedVolts);
    } else {
      velocityOfIntakeSpeedRPS = velocityMotor.getVelocity();
      statorCurrentOfIntakeSpeedAmps = velocityMotor.getStatorCurrent();
      outputOfIntakeSpeedVolts = velocityMotor.getMotorVoltage();
      accelerationOfIntakeSpeed = velocityMotor.getAcceleration();
      intakeVelocityConfig.configureSignals(velocityMotor, 50.0, velocityOfIntakeSpeedRPS,
          statorCurrentOfIntakeSpeedAmps, outputOfIntakeSpeedVolts, accelerationOfIntakeSpeed);
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

  public void highTelemetry(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        accelerationOfIntakePosition,
        accelerationOfIntakeSpeed, outputOfIntakePositionVolts, outputOfIntakeSpeedVolts);
    inputs.accelerationOfIntakePosition = accelerationOfIntakePosition.getValue().in(RotationsPerSecondPerSecond);
    inputs.rightAccelerationOfIntakeSpeed = accelerationOfIntakeSpeed.getValue().in(RotationsPerSecondPerSecond);
    inputs.outputOfIntakePositionVolts = outputOfIntakePositionVolts.getValue().in(Volts);
    inputs.rightOutputOfIntakeSpeedVolts = outputOfIntakeSpeedVolts.getValue()
        .in(Volts);
    lowTelemetry(inputs);

  }

  public void lowTelemetry(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocityOfIntakePositionRPS, statorCurrentOfIntakePositionAmps,
        velocityOfIntakeSpeedRPS, statorCurrentOfIntakeSpeedAmps);
    inputs.velocityOfIntakePositionRPS = velocityOfIntakePositionRPS.getValue().in(Rotation.per(Minute));
    inputs.statorCurrentOfIntakePositionAmps = statorCurrentOfIntakePositionAmps.getValue().in(Amps);
    inputs.rightVelocityMotorConnected = velocityMotor.isConnected();
    inputs.rightVelocityOfIntakeSpeedRPS = velocityOfIntakeSpeedRPS.getValue().in(Rotation.per(Minute));
    inputs.rightStatorCurrentOfIntakeSpeedAmps = statorCurrentOfIntakeSpeedAmps.getValue().in(Amps);
    inputs.positionConnected = velocityMotor.isConnected();
    inputs.intakePosition = positionMotor.getPosition().getValueAsDouble();
  }

  public void setVelocity(double velocity) {
    intakeVelocitySetpoint = velocity;
    // velocityMotor.setControl(requestVelocity.withVelocity(velocity).withFeedForward(0.6));
    velocityMotor.set(1);
  }

  public void setVelocity(IntakeState desiredState) {
    setVelocity(desiredState.getSpeed());
  }

  public void setPosition(IntakeState desiredState) {
    setPosition(desiredState.getPosition());
  }

  public void setPosition(double pos) {
    intakePivotSetpoint = pos;

    positionMotor.setControl(requestPositionVoltage.withPosition(pos).withFeedForward(.8));

  }

  public void retractIntake() {
    intakePivotSetpoint = 0;
    positionMotor.setControl(requestPositionVoltage.withPosition(0).withFeedForward(-.7));
  }

  public double getVelocity() {
    return velocityMotor.getVelocity().getValueAsDouble();
  }

  public void resetEncoder() {
    positionMotor.setPosition(0);
  }

  @Override
  public void stop() {
    stopRollerWheel();
    stopPivotMotor();
  }

  public void stopRollerWheel() {
    stopRightRollerWheel();
    stopLeftRollerWheel();
  }
    public void stopLeftRollerWheel() {
  }
    public void stopRightRollerWheel() {
    intakeVelocitySetpoint = 0.0;
    velocityMotor.stopMotor();
  }

  public void stopPivotMotor() {
    intakePivotSetpoint = 0.0;
    velocityMotor.stopMotor();
  }

  public void stopBottom() {
  }

  @Override
  public void periodic() {

  }

  public void simulationPeriodic() {
  }

  /* Characterization */
  public void runCharacterization_IntakeVelocity(double output) {
    velocityMotor.setControl(switch (CharacterizationClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
  }

  /* Characterization */
  public void runCharacterization_IntakePosition(double output) {
    velocityMotor.setControl(switch (CharacterizationClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity_Intake() {
    double avg = (velocityMotor.getVelocity().getValue().in(RotationsPerSecond)) / 1;
    return avg;
  }

  /** Returns the module position. */
  public double getFFCharacterizationPosition_Intake() {
    double avg = (positionMotor.getPosition().getValue().in(Rotations)) / 1;
    return avg;
  }

  public void setNeturalCoast() {
    intakePivotConfig = new ModuleConfigurator(pivotMotorGains.toSlot0Configs(),
        Constants.IntakeConstants.PivotConstants.pivotMotorId,
        Constants.IntakeConstants.PivotConstants.isInverted,
        true,
        Constants.IntakeConstants.PivotConstants.currentLimit,
        Constants.IntakeConstants.PivotConstants.peakForwardLimit,
        Constants.IntakeConstants.PivotConstants.peakReverseLimit);
    intakePivotConfig.configureMotor(positionMotor, pivotMotorGains);
  }
    public void setNeturalBrake() {
    intakePivotConfig = new ModuleConfigurator(pivotMotorGains.toSlot0Configs(),
        Constants.IntakeConstants.PivotConstants.pivotMotorId,
        Constants.IntakeConstants.PivotConstants.isInverted,
        false,
        Constants.IntakeConstants.PivotConstants.currentLimit,
        Constants.IntakeConstants.PivotConstants.peakForwardLimit,
        Constants.IntakeConstants.PivotConstants.peakReverseLimit);
    intakePivotConfig.configureMotor(positionMotor, pivotMotorGains);
  }
}
