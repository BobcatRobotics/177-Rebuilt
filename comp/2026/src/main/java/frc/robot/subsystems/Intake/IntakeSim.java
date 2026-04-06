package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volt;
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

public class IntakeSim implements IntakeIO {

  private TalonFX positionMotor;
  public ModuleConfigurator intakePivotConfig;
  private SimMotorFX positionMotorSim;

  private TalonFX leftVelocityMotor;
  public ModuleConfigurator leftintakeVelocityConfig;
  private SimMotorFX leftVelocityMotorSim;

  private TalonFX rightVelocityMotor;
  public ModuleConfigurator rightIntakeVelocityConfig;
  private SimMotorFX rightVelocityMotorSim;

  private TorqueCurrentFOC characterizationRequestTorqueCurrentFOC = new TorqueCurrentFOC(0);
  private VoltageOut characterizationRequestVoltage = new VoltageOut(0);
  private final VelocityVoltage requestVelocity = new VelocityVoltage(0);
  private final PositionTorqueCurrentFOC requestPosition = new PositionTorqueCurrentFOC(0);
  private final PositionVoltage requestPositionVoltage = new PositionVoltage(0);

  // private TunablePID intakePivotPID;
  // private TunablePID intakeVelocityPID;

  public double intakePivotSetpoint = 0;
  public double intakeVelocitySetpoint = 0;

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

  // private FindLimit seekLowerRange;
  Gains pivotMotorGains;
  Gains rightRollerMotorGains;
  Gains leftRollerMotorGains;

  public IntakeSim() {
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

    // seekLowerRange = new FindLimit(false, positionMotor);
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
    rightVelocityMotorSim = new SimMotorFX(rightVelocityMotor, Constants.IntakeConstants.RightRollerConstants.isInverted);
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
    leftVelocityMotorSim = new SimMotorFX(leftVelocityMotor, Constants.IntakeConstants.LeftRollerConstants.isInverted);
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
    positionMotorSim = new SimMotorFX(positionMotor, Constants.IntakeConstants.PivotConstants.isInverted);

  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    positionMotorSim.update();
    leftVelocityMotorSim.update();
    rightVelocityMotorSim.update();
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
    inputs.rightAccelerationOfIntakeSpeed = rightAccelerationOfIntakeSpeed.getValue().in(RotationsPerSecondPerSecond);
    inputs.leftOutputOfIntakeSpeedVolts = leftOutputOfIntakeSpeedVolts.getValue().in(Volts);
    inputs.rightOutputOfIntakeSpeedVolts = rightOutputOfIntakeSpeedVolts.getValue().in(Volts);

    lowTelemetry(inputs);
  }

  public void setVelocity(double velocity) {
    intakeVelocitySetpoint = velocity;
    // velocityMotor.setControl(requestVelocity.withVelocity(velocity).withFeedForward(0.6));
    rightVelocityMotor.set(1);
    leftVelocityMotor.set(1);
    rightVelocityMotorSim.set(1);
    leftVelocityMotorSim.set(1);
  }
    public void manualReverseIntake(double velocity) {
    intakeVelocitySetpoint = velocity;
    // velocityMotor.setControl(requestVelocity.withVelocity(velocity).withFeedForward(0.6));
    rightVelocityMotor.set(velocity);
    leftVelocityMotor.set(velocity);
    rightVelocityMotorSim.setVelocity(velocity);
    leftVelocityMotorSim.setVelocity(velocity);
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
    positionMotorSim.setPosition(pos);
  }

  public void retractIntake() {
    intakePivotSetpoint = 0;
    positionMotor.setControl(requestPositionVoltage.withPosition(0).withFeedForward(-.7));
    positionMotorSim.setPosition(0);
  }

  public void manualRetractIntake() {
    positionMotor.set(-.2);
    positionMotorSim.set(-0.2);
  }

  public double getRightVelocity() {
    return rightVelocityMotor.getVelocity().getValueAsDouble();
  }

  public double getLeftVelocity() {
    return leftVelocityMotor.getVelocity().getValueAsDouble();
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
    stopLeftRollerWheel();
    stopRightRollerWheel();
  }

  public void stopLeftRollerWheel() {
    intakeVelocitySetpoint = 0.0;
    leftVelocityMotor.stopMotor();
    rightVelocityMotor.stopMotor();
    rightVelocityMotorSim.setVelocity(0);
    leftVelocityMotorSim.setVelocity(0);
  }


  public void stopRightRollerWheel() {
    intakeVelocitySetpoint = 0.0;
    leftVelocityMotor.stopMotor();
    rightVelocityMotor.stopMotor();
    rightVelocityMotorSim.setVelocity(0);
    leftVelocityMotorSim.setVelocity(0);
  }

  public void stopPivotMotor() {
    intakePivotSetpoint = 0.0;
    positionMotor.stopMotor();
    positionMotorSim.setVelocity(0);
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
    rightVelocityMotor.setControl(switch (CharacterizationClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
    leftVelocityMotor.setControl(switch (CharacterizationClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
  }

  /* Characterization */
  public void runCharacterization_IntakePosition(double output) {
    positionMotor.setControl(switch (CharacterizationClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity_Intake() {
    double avg = (leftVelocityMotor.getVelocity().getValue().in(RotationsPerSecond)
        + rightVelocityMotor.getVelocity().getValue().in(RotationsPerSecond)) / 2;
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
