package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.bobcatrobotics.Hardware.Characterization.CharacterizationClosedLoopOutputType;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Modules.ModuleConfigurator;
import frc.robot.subsystems.Shooter.Modules.TunablePID;
import frc.robot.util.Gains;

public class ShooterRealDual implements ShooterIO {
  private TalonFX shooterFlywheelInnerLeft;
  public ModuleConfigurator flywheelConfigLeft;
  private TalonFX shooterFlywheelInnerRight;
  public ModuleConfigurator flywheelConfigRight;

  private TalonFX shooterIntakeMotor;
  public ModuleConfigurator intakeWheelConfig;
  private TalonFX backspinWheelMotorLeft;
  public ModuleConfigurator backspinMConfigLeft;
  private TalonFX backspinWheelMotorRight;
  public ModuleConfigurator backspinMConfigRight;

  // Defines tunable values , particularly for configurations of motors ( IE PIDs
  // )
  private VelocityTorqueCurrentFOC velIntakeRequest = new VelocityTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC velShooterLeftRequest = new VelocityTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC velShooterRightRequest = new VelocityTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC velBackspinLeftRequest = new VelocityTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC velBackspinRightRequest = new VelocityTorqueCurrentFOC(0);

  private TorqueCurrentFOC characterizationRequestTorqueCurrentFOC = new TorqueCurrentFOC(0);
  private VoltageOut characterizationRequestVoltage = new VoltageOut(0);

  private StatusSignal<AngularVelocity> velocityOfMainFlywhelLeftRPS;
  private StatusSignal<Current> statorCurrentOfMainFlywheelLeftAmps;
  private StatusSignal<Voltage> outputOfMainFlywheelLeftVolts;
  private StatusSignal<AngularAcceleration> accelerationOfMainFlywheelLeft;
  private StatusSignal<AngularVelocity> velocityOfMainFlywheelRightRPS;
  private StatusSignal<Current> statorCurrentOfMainFlywheelRightAmps;
  private StatusSignal<Voltage> outputOfMainFlywheelRightVolts;
  private StatusSignal<AngularAcceleration> accelerationOfMainFlywheelRight;

  private StatusSignal<AngularVelocity> velocityOfIntakeRPS;
  private StatusSignal<Current> statorCurrentOfIntakeAmps;
  private StatusSignal<Voltage> outputOfIntakeVolts;
  private StatusSignal<AngularAcceleration> accelerationOfIntake;

  private StatusSignal<AngularVelocity> velocityOfbackspinWheelMotorLeftRPS;
  private StatusSignal<Current> statorCurrentOfBackspinLeftAmps;
  private StatusSignal<Voltage> outputOfBackspinLeftVolts;
  private StatusSignal<AngularAcceleration> accelerationOfBackspinLeft;
  private StatusSignal<AngularVelocity> velocityOfbackspinWheelMotorRightRPS;
  private StatusSignal<Current> statorCurrentOfBackspinRightAmps;
  private StatusSignal<Voltage> outputOfBackspinRightVolts;
  private StatusSignal<AngularAcceleration> accelerationOfBackspinRight;

  public double mainFlywheelSetpoint = 0;
  public double intakeSetpoint = 0;
  public double backspinSetpointRight = 0;
  public double backspinSetpointLeft = 0;

  private TunablePID flywheelLeftPID;
  private TunablePID flywheelRighPID;
  private TunablePID intakePID;
  private TunablePID backspinLeftPID;
  private TunablePID backspinRightPID;

  public ShooterRealDual() {
    // Flywheel Configuration
    Gains flywheelGains = new Gains.Builder()
        .kP(Constants.ShooterConstants.SharedFlywheel.kshooterMainkP)
        .kI(Constants.ShooterConstants.SharedFlywheel.kshooterMainkI)
        .kD(Constants.ShooterConstants.SharedFlywheel.kshooterMainkD)
        .kS(Constants.ShooterConstants.SharedFlywheel.kshooterMainkS)
        .kV(Constants.ShooterConstants.SharedFlywheel.kshooterMainkV)
        .kA(Constants.ShooterConstants.SharedFlywheel.kshooterMainkA).build();
    Gains intakeGains = new Gains.Builder()
        .kP(Constants.ShooterConstants.SharedIntake.kIntakeMotorkP)
        .kI(Constants.ShooterConstants.SharedIntake.kIntakeMotorkI)
        .kD(Constants.ShooterConstants.SharedIntake.kIntakeMotorkD)
        .kS(Constants.ShooterConstants.SharedIntake.kIntakeMotorkS)
        .kV(Constants.ShooterConstants.SharedIntake.kIntakeMotorkV)
        .kA(Constants.ShooterConstants.SharedIntake.kIntakeMotorkA).build();
    Gains backspinLeftGains = new Gains.Builder()
        .kP(Constants.ShooterConstants.Left.kBackspinMotorkP)
        .kI(Constants.ShooterConstants.Left.kBackspinMotorkI)
        .kD(Constants.ShooterConstants.Left.kBackspinMotorkD)
        .kS(Constants.ShooterConstants.Left.kBackspinMotorkS)
        .kV(Constants.ShooterConstants.Left.kBackspinMotorkV)
        .kA(Constants.ShooterConstants.Left.kBackspinMotorkA).build();
    Gains backspinRightGains = new Gains.Builder()
        .kP(Constants.ShooterConstants.Right.kBackspinMotorkP)
        .kI(Constants.ShooterConstants.Right.kBackspinMotorkI)
        .kD(Constants.ShooterConstants.Right.kBackspinMotorkD)
        .kS(Constants.ShooterConstants.Right.kBackspinMotorkS)
        .kV(Constants.ShooterConstants.Right.kBackspinMotorkV)
        .kA(Constants.ShooterConstants.Right.kBackspinMotorkA).build();

    setupLeftFlywheel(flywheelGains);
    setupRightFlywheel(flywheelGains);
    setupIntake(intakeGains);
    setupLeftBackspin(backspinLeftGains);
    setupRightBackspin(backspinRightGains);
  }

  public void setupLeftFlywheel(Gains g) {
    flywheelLeftPID = new TunablePID(
        "/Shooter/Flywheel/Left/PID", g);
    flywheelConfigLeft = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.SharedFlywheel.FlywheelInnerIDLeft,
        Constants.ShooterConstants.SharedFlywheel.isInvertedLeft,
        Constants.ShooterConstants.SharedFlywheel.isCoastRight,
        Constants.ShooterConstants.SharedFlywheel.currentLimit);
    shooterFlywheelInnerLeft = new TalonFX(flywheelConfigLeft.getMotorInnerId(), new CANBus("rio"));
    flywheelConfigLeft.configureMotor(shooterFlywheelInnerLeft, flywheelLeftPID);
    velocityOfMainFlywhelLeftRPS = shooterFlywheelInnerLeft.getVelocity();
    statorCurrentOfMainFlywheelLeftAmps = shooterFlywheelInnerLeft.getStatorCurrent();
    outputOfMainFlywheelLeftVolts = shooterFlywheelInnerLeft.getMotorVoltage();
    accelerationOfMainFlywheelLeft = shooterFlywheelInnerLeft.getAcceleration();
    flywheelConfigLeft.configureSignals(shooterFlywheelInnerLeft, 50.0, velocityOfMainFlywhelLeftRPS,
        statorCurrentOfMainFlywheelLeftAmps, outputOfMainFlywheelLeftVolts, accelerationOfMainFlywheelLeft);
  }

  public void setupRightFlywheel(Gains g) {
    flywheelRighPID = new TunablePID(
        "/Shooter/Flywheel/Right/PID", g);
    // Flywheel Configuration
    flywheelConfigRight = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.SharedFlywheel.FlywheelInnerIDRight,
        Constants.ShooterConstants.SharedFlywheel.isInvertedRight,
        Constants.ShooterConstants.SharedFlywheel.isCoastRight,
        Constants.ShooterConstants.SharedFlywheel.currentLimit);
    shooterFlywheelInnerRight = new TalonFX(flywheelConfigRight.getMotorInnerId(), new CANBus("rio"));
    flywheelConfigRight.configureMotor(shooterFlywheelInnerRight, flywheelRighPID);
    velocityOfMainFlywheelRightRPS = shooterFlywheelInnerRight.getVelocity();
    statorCurrentOfMainFlywheelRightAmps = shooterFlywheelInnerRight.getStatorCurrent();
    outputOfMainFlywheelRightVolts = shooterFlywheelInnerRight.getMotorVoltage();
    accelerationOfMainFlywheelRight = shooterFlywheelInnerRight.getAcceleration();
    flywheelConfigRight.configureSignals(shooterFlywheelInnerRight, 50.0, velocityOfMainFlywheelRightRPS,
        statorCurrentOfMainFlywheelRightAmps, outputOfMainFlywheelRightVolts, accelerationOfMainFlywheelRight);
  }


  public void setupIntake(Gains g) {
    // Intake Configuration
    intakePID = new TunablePID(
        "/Shooter/Intake/PID", g);
    intakeWheelConfig = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.SharedIntake.intakeIDLeft,
        Constants.ShooterConstants.SharedIntake.isInverted,
        Constants.ShooterConstants.SharedIntake.isCoast,
        Constants.ShooterConstants.SharedIntake.currentLimit);
    shooterIntakeMotor = new TalonFX(intakeWheelConfig.getMotorInnerId(), new CANBus("rio"));
    intakeWheelConfig.configureMotor(shooterIntakeMotor, intakePID);
    velocityOfIntakeRPS = shooterIntakeMotor.getVelocity();
    statorCurrentOfIntakeAmps = shooterIntakeMotor.getStatorCurrent();
    outputOfIntakeVolts = shooterIntakeMotor.getMotorVoltage();
    accelerationOfIntake = shooterIntakeMotor.getAcceleration();
    intakeWheelConfig.configureSignals(shooterIntakeMotor, 50.0, velocityOfIntakeRPS,
        statorCurrentOfIntakeAmps, outputOfIntakeVolts, accelerationOfIntake);
  }

  public void setupLeftBackspin(Gains g) {
    backspinLeftPID = new TunablePID(
        "/Shooter/Backspin/Left/PID", g);
    // Flywheel Configuration
    backspinMConfigLeft = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.Left.BackspinID,
        Constants.ShooterConstants.Left.isInverted,
        Constants.ShooterConstants.Left.isCoast,
        Constants.ShooterConstants.Left.currentLimit);
    backspinWheelMotorLeft = new TalonFX(backspinMConfigLeft.getMotorInnerId(), new CANBus("rio"));
    backspinMConfigLeft.configureMotor(backspinWheelMotorLeft, backspinLeftPID);
    velocityOfbackspinWheelMotorLeftRPS = backspinWheelMotorLeft.getVelocity();
    statorCurrentOfBackspinLeftAmps = backspinWheelMotorLeft.getStatorCurrent();
    outputOfBackspinLeftVolts = backspinWheelMotorLeft.getMotorVoltage();
    accelerationOfBackspinLeft = backspinWheelMotorLeft.getAcceleration();
    backspinMConfigLeft.configureSignals(backspinWheelMotorLeft, 50.0, velocityOfbackspinWheelMotorLeftRPS,
        statorCurrentOfBackspinLeftAmps, outputOfBackspinLeftVolts, accelerationOfBackspinLeft);
  }

  public void setupRightBackspin(Gains g) {
    backspinRightPID = new TunablePID(
        "/Shooter/Backspin/Right/PID", g);
    // Flywheel Configuration
    backspinMConfigRight = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.Right.BackspinID,
        Constants.ShooterConstants.Right.isInverted,
        Constants.ShooterConstants.Right.isCoast,
        Constants.ShooterConstants.Right.currentLimit);
    backspinWheelMotorRight = new TalonFX(backspinMConfigRight.getMotorInnerId(), new CANBus("rio"));
    backspinMConfigRight.configureMotor(backspinWheelMotorRight, backspinRightPID);
    velocityOfbackspinWheelMotorRightRPS = backspinWheelMotorRight.getVelocity();
    statorCurrentOfBackspinRightAmps = backspinWheelMotorRight.getStatorCurrent();
    outputOfBackspinRightVolts = backspinWheelMotorRight.getMotorVoltage();
    accelerationOfBackspinRight = backspinWheelMotorRight.getAcceleration();
    flywheelConfigLeft.configureSignals(backspinWheelMotorRight, 50.0, velocityOfbackspinWheelMotorRightRPS,
        statorCurrentOfBackspinRightAmps, outputOfBackspinRightVolts, accelerationOfBackspinRight);
  }

  public void updateInputs(ShooterIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        velocityOfMainFlywhelLeftRPS,
        velocityOfMainFlywheelRightRPS,
        velocityOfbackspinWheelMotorLeftRPS,
        velocityOfbackspinWheelMotorRightRPS,
        velocityOfIntakeRPS,
        accelerationOfMainFlywheelLeft,
        accelerationOfMainFlywheelRight,
        accelerationOfBackspinLeft,
        accelerationOfBackspinRight,
        accelerationOfIntake,
        statorCurrentOfBackspinLeftAmps,
        statorCurrentOfBackspinRightAmps,
        statorCurrentOfMainFlywheelLeftAmps,
        statorCurrentOfMainFlywheelRightAmps,
        outputOfBackspinLeftVolts,
        outputOfBackspinRightVolts,
        outputOfMainFlywheelLeftVolts,
        outputOfMainFlywheelRightVolts,
        outputOfIntakeVolts);

    inputs.velocityOfMainFlywheelLeftRPS = velocityOfMainFlywhelLeftRPS.getValue().in(Rotations.per(Seconds));
    inputs.velocityOfMainFlywheelRightRPS = velocityOfMainFlywheelRightRPS.getValue().in(Rotations.per(Seconds));
    inputs.velocityOfbackspinWheelMotorLeftRPS = velocityOfbackspinWheelMotorLeftRPS.getValue()
        .in(Rotations.per(Seconds));
    inputs.velocityOfbackspinWheelMotorRightRPS = velocityOfbackspinWheelMotorRightRPS.getValue()
        .in(Rotations.per(Seconds));
    inputs.velocityOfIntakeRPS = velocityOfIntakeRPS.getValue().in(Rotations.per(Seconds));
    inputs.accelerationOfMainFlywheelLeft = accelerationOfMainFlywheelLeft.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.accelerationOfMainFlywheelRight = accelerationOfMainFlywheelRight.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.accelerationOfBackspinLeft = accelerationOfBackspinLeft.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.accelerationOfIntake = accelerationOfIntake.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.statorCurrentOfBackspinLeftAmps = statorCurrentOfBackspinLeftAmps.getValue().in(Amps);
    inputs.statorCurrentOfBackspinRightAmps = statorCurrentOfBackspinRightAmps.getValue().in(Amps);
    inputs.statorCurrentOfMainFlywheelLeftAmps = statorCurrentOfMainFlywheelLeftAmps.getValue().in(Amps);
    inputs.statorCurrentOfMainFlywheelRightAmps = statorCurrentOfMainFlywheelRightAmps.getValue().in(Amps);
    inputs.statorCurrentOfIntakeAmps = statorCurrentOfIntakeAmps.getValue().in(Amps);

    inputs.backspinWheelMotorRightConnected = backspinWheelMotorRight.isConnected();
    inputs.backspinWheelMotorLeftConnected = backspinWheelMotorLeft.isConnected();
    inputs.shooterFlywheelInnerLeftConnected = shooterFlywheelInnerLeft.isConnected();
    inputs.shooterFlywheelInnerRightConnected = shooterFlywheelInnerRight.isConnected();
    inputs.shooterIntakeMotorConnected = shooterIntakeMotor.isConnected();

    inputs.outputOfBackspinLeftVolts = outputOfBackspinLeftVolts.getValue().in(Volts);
    inputs.outputOfBackspinRightVolts = outputOfBackspinRightVolts.getValue().in(Volts);
    inputs.outputOfMainFlywheelLeftVolts = outputOfMainFlywheelLeftVolts.getValue().in(Volts);
    inputs.outputOfMainFlywheelRightVolts = outputOfMainFlywheelRightVolts.getValue().in(Volts);
    inputs.outputOfIntakeVolts = outputOfIntakeVolts.getValue().in(Volts);

  }

  public void setOutput(double shooterOutput, double backspinOutputLeft, double backspinOutputRight) {
    shooterFlywheelInnerLeft.set(shooterOutput);
    shooterFlywheelInnerRight.set(shooterOutput);
    backspinWheelMotorLeft.set(backspinOutputLeft);
    backspinWheelMotorRight.set(backspinOutputRight);
  }

  public void setVelocity(ShooterState desiredState) {
    setVelocity(desiredState.getFlywheelSpeed(),
        desiredState.getBackspinSpeedOfLeft(),
        desiredState.getBackspinSpeedOfRight(), desiredState.getIntakeSpeed());
  }

  public void setVelocity(double shooterFlywheelSpeed, double shooterBackspinSpeedOfLeft,
      double shooterBackspinSpeedOfRight, double shooterIntakeSpeed) {
    setMainWheelSpeed(shooterFlywheelSpeed);
    setBackspinSpeedOfLeft(shooterBackspinSpeedOfLeft);
    setBackspinSpeedOfRight(shooterBackspinSpeedOfRight);
    setIntakeSpeed(shooterIntakeSpeed);
  }

  public void setMainWheelSpeed(double shooterFlywheelSpeedInRPS) {
    mainFlywheelSetpoint = shooterFlywheelSpeedInRPS;
    shooterFlywheelInnerLeft.setControl(velShooterLeftRequest.withVelocity(mainFlywheelSetpoint));
    shooterFlywheelInnerRight.setControl(velShooterRightRequest.withVelocity(mainFlywheelSetpoint));
  }

  public void setBackspinSpeedOfLeft(double shooterBackspinSpeedInRPS) {
    backspinSetpointLeft = shooterBackspinSpeedInRPS;
    backspinWheelMotorLeft.setControl(velBackspinLeftRequest.withVelocity(backspinSetpointLeft));
  }

  public void setBackspinSpeedOfRight(double shooterBackspinSpeedInRPS) {
    backspinSetpointRight = shooterBackspinSpeedInRPS;
    backspinWheelMotorRight.setControl(velBackspinRightRequest.withVelocity(backspinSetpointRight));
  }

  public void setIntakeSpeed(double shooterIntakeSpeedInRPS) {
    intakeSetpoint = shooterIntakeSpeedInRPS;
    shooterIntakeMotor.setControl(velIntakeRequest.withVelocity(intakeSetpoint));
  }

  public void holdPosition() {
  }

  public void stopMainWheel() {
    mainFlywheelSetpoint = 0;
    shooterFlywheelInnerLeft.stopMotor();
    shooterFlywheelInnerRight.stopMotor();
  }

  public void stopBackspinLeftWheel() {
    backspinSetpointLeft = 0;
    backspinWheelMotorLeft.stopMotor();
  }

  public void stopBackspinRightWheel() {
    backspinSetpointRight = 0;
    backspinWheelMotorRight.stopMotor();
  }

  public void stopIntakeMotor() {
    intakeSetpoint = 0;
    shooterIntakeMotor.stopMotor();

  }

  @Override
  public void periodic() {
    if (flywheelLeftPID.hasChanged()) {
      flywheelConfigLeft.updateMotorPID(shooterFlywheelInnerLeft, flywheelLeftPID);
    }
    if (flywheelRighPID.hasChanged()) {
      flywheelConfigRight.updateMotorPID(shooterFlywheelInnerRight, flywheelRighPID);
    }

    if (backspinLeftPID.hasChanged()) {
      backspinMConfigLeft.updateMotorPID(backspinWheelMotorRight, backspinLeftPID);
    }
    if (backspinRightPID.hasChanged()) {
      backspinMConfigRight.updateMotorPID(backspinWheelMotorRight, backspinRightPID);
    }
    if (intakePID.hasChanged()) {
      intakeWheelConfig.updateMotorPID(shooterIntakeMotor, intakePID);
    }
  }

  public void simulationPeriodic() {
  }

  /* Characterization */
  public void runCharacterization_Flywheel(double output) {
    shooterFlywheelInnerLeft.setControl(switch (ClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
    shooterFlywheelInnerRight.setControl(switch (ClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity_Flywheel() {
    double avg = (shooterFlywheelInnerLeft.getVelocity().getValue().in(RotationsPerSecond) +
        shooterFlywheelInnerRight.getVelocity().getValue().in(RotationsPerSecond) ) / 2;
    return avg;
  }

  /* Characterization */
  public void runCharacterization_Backspin(double output) {
    backspinWheelMotorLeft.setControl(switch (CharacterizationClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
    backspinWheelMotorRight.setControl(switch (CharacterizationClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity_Backspin() {
    double avg = (
        backspinWheelMotorLeft.getVelocity().getValue().in(RotationsPerSecond) +
        backspinWheelMotorRight.getVelocity().getValue().in(RotationsPerSecond)) / 2;
    return avg;
  }

  /* Characterization */
  public void runCharacterization_Intake(double output) {
    shooterIntakeMotor.setControl(switch (CharacterizationClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity_Intake() {
    double avg = shooterIntakeMotor.getVelocity().getValue().in(RotationsPerSecond);
    return avg;
  }
}