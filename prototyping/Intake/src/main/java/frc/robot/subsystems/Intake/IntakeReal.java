package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.bobcatrobotics.Hardware.Characterization.CharacterizationClosedLoopOutputType;
import org.bobcatrobotics.Hardware.Motors.FindLimit;
import org.bobcatrobotics.Util.Tunables.Gains;
import org.bobcatrobotics.Util.Tunables.TunablePID;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.Modules.ModuleConfigurator;

public class IntakeReal implements IntakeIO {

  private TalonFX positionMotor;
  public ModuleConfigurator intakePivotConfig;

  private TalonFX velocityMotor;
  public ModuleConfigurator intakeVelocityConfig;

  private TorqueCurrentFOC characterizationRequestTorqueCurrentFOC = new TorqueCurrentFOC(0);
  private VoltageOut characterizationRequestVoltage = new VoltageOut(0);
  private final VelocityTorqueCurrentFOC requestVelocity = new VelocityTorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC requestPosition = new PositionTorqueCurrentFOC(0);

  private TunablePID intakePivotPID;
  private TunablePID intakeVelocityPID;

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


  private FindLimit seekLowerRange;

  public IntakeReal() {
    Gains pivotMotorGains = new Gains.Builder()
        .kP(Constants.IntakeConstants.PivotConstants.kP)
        .kI(Constants.IntakeConstants.PivotConstants.kI)
        .kD(Constants.IntakeConstants.PivotConstants.kD)
        .kS(Constants.IntakeConstants.PivotConstants.kS)
        .kV(Constants.IntakeConstants.PivotConstants.kV)
        .kA(Constants.IntakeConstants.PivotConstants.kA).build();
    Gains rollerMotorGains = new Gains.Builder()
        .kP(Constants.IntakeConstants.RollerConstants.kP)
        .kI(Constants.IntakeConstants.RollerConstants.kI)
        .kD(Constants.IntakeConstants.RollerConstants.kD)
        .kS(Constants.IntakeConstants.RollerConstants.kS)
        .kV(Constants.IntakeConstants.RollerConstants.kV)
        .kA(Constants.IntakeConstants.RollerConstants.kA).build();
    setupRollerMotor(rollerMotorGains);
    setupPivotMotor(rollerMotorGains);

    seekLowerRange = new FindLimit(false, positionMotor);
  }

  public void setupRollerMotor(Gains g) {
    intakeVelocityPID = new TunablePID(
        "/Hopper/Top/PID", g);
    intakeVelocityConfig = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.IntakeConstants.RollerConstants.rollerMotorId,
        Constants.IntakeConstants.RollerConstants.isInverted,
        Constants.IntakeConstants.RollerConstants.isCoast,
        Constants.IntakeConstants.RollerConstants.currentLimit);
    velocityMotor = new TalonFX(intakeVelocityConfig.getMotorInnerId(), new CANBus("rio"));
    intakeVelocityConfig.configureMotor(velocityMotor, intakeVelocityPID);
    velocityOfIntakeSpeedRPS = velocityMotor.getVelocity();
    statorCurrentOfIntakeSpeedAmps = velocityMotor.getStatorCurrent();
    outputOfIntakeSpeedVolts = velocityMotor.getMotorVoltage();
    accelerationOfIntakeSpeed = velocityMotor.getAcceleration();
    intakeVelocityConfig.configureSignals(velocityMotor, 50.0, velocityOfIntakeSpeedRPS,
        statorCurrentOfIntakeSpeedAmps, outputOfIntakeSpeedVolts, accelerationOfIntakeSpeed);
  }

  public void setupPivotMotor(Gains g) {
    intakePivotPID = new TunablePID(
        "/Hopper/Top/PID", g);
    intakePivotConfig = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.IntakeConstants.PivotConstants.pivotMotorId,
        Constants.IntakeConstants.PivotConstants.isInverted,
        Constants.IntakeConstants.PivotConstants.isCoast,
        Constants.IntakeConstants.PivotConstants.currentLimit);
    positionMotor = new TalonFX(intakePivotConfig.getMotorInnerId(), new CANBus("rio"));
    intakePivotConfig.configureMotor(positionMotor, intakePivotPID);
    velocityOfIntakePositionRPS = positionMotor.getVelocity();
    statorCurrentOfIntakePositionAmps = positionMotor.getStatorCurrent();
    outputOfIntakePositionVolts = positionMotor.getMotorVoltage();
    accelerationOfIntakePosition = positionMotor.getAcceleration();
    intakePivotConfig.configureSignals(positionMotor, 50.0, velocityOfIntakePositionRPS,
        statorCurrentOfIntakePositionAmps, outputOfIntakePositionVolts, accelerationOfIntakePosition);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocityOfIntakePositionRPS, statorCurrentOfIntakePositionAmps, outputOfIntakePositionVolts,
        accelerationOfIntakePosition,
        velocityOfIntakeSpeedRPS, statorCurrentOfIntakeSpeedAmps, outputOfIntakeSpeedVolts, accelerationOfIntakeSpeed);

    inputs.velocityOfIntakePositionRPS = velocityOfIntakePositionRPS.getValue().in(Rotation.per(Minute));
    inputs.statorCurrentOfIntakePositionAmps = statorCurrentOfIntakePositionAmps.getValue().in(Amps);
    inputs.outputOfIntakePositionVolts = outputOfIntakePositionVolts.getValue().in(Volts);
    inputs.accelerationOfIntakePosition = accelerationOfIntakePosition.getValue().in(RotationsPerSecondPerSecond);
    inputs.velocityConnected = velocityMotor.isConnected();

    inputs.velocityOfIntakeSpeedRPS = velocityOfIntakeSpeedRPS.getValue().in(Rotation.per(Minute));
    inputs.statorCurrentOfIntakeSpeedAmps = statorCurrentOfIntakeSpeedAmps.getValue().in(Amps);
    inputs.outputOfIntakeSpeedVolts = outputOfIntakeSpeedVolts.getValue()
        .in(Volts);
    inputs.accelerationOfIntakeSpeed = accelerationOfIntakeSpeed.getValue().in(RotationsPerSecondPerSecond);
    inputs.positionConnected = velocityMotor.isConnected();

  }

  public void setVelocity(double velocity) {
    intakeVelocitySetpoint = velocity;
    requestVelocity.withVelocity(velocity);
  }

  public void setVelocity(IntakeState desiredState) {
    setVelocity(desiredState.getSpeed());
  }

  public void setPosition(IntakeState desiredState) {
    setPosition(desiredState.getPosition());
  }

  public void setPosition(double pos) {
    intakeVelocitySetpoint = pos;
    requestPosition.withPosition(pos);
  }

  public double getVelocity() {
    return velocityMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void stop() {
    stopRollerWheel();
    stopPivotMotor();
  }

  public void stopRollerWheel() {
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
    if (intakePivotPID.hasChanged()) {
      intakePivotConfig.updateMotorPID(positionMotor, intakePivotPID);
    }
    if (intakeVelocityPID.hasChanged()) {
      intakeVelocityConfig.updateMotorPID(velocityMotor, intakeVelocityPID);
    }

  }
  
    public void simulationPeriodic() {
  }

  /* Characterization */
  public void runCharacterization_Intake(double output) {
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

  /**
   * Seeks the hard stop. This slowly drives the motor up/down based on
   * initialization parameters.
   * until we see a drop in velocity and a spike in stator current,
   * indicating that we've hit a hard stop.
   *
   * @return Command to run
   */
  public Command findLowerLimit() {
    return seekLowerRange.findLimit();
  }
}
