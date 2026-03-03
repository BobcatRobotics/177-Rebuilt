package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.bobcatrobotics.Hardware.Characterization.CharacterizationClosedLoopOutputType;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Modules.ModuleConfigurator;
import org.bobcatrobotics.Util.Tunables.Gains;
import org.bobcatrobotics.Util.Tunables.TunablePID;

public class ShooterRealQuad implements ShooterIO {
  private TalonFX shooterFlywheelInnerLeft;
  public ModuleConfigurator flywheelConfigLeft;
  private TalonFX shooterFlywheelInnerRight;
  public ModuleConfigurator flywheelConfigRight;
  private TalonFX shooterFlywheelOuterRight;
  public ModuleConfigurator flywheelConfigOuterRight;
  private TalonFX shooterFlywheelOuterLeft;
  public ModuleConfigurator flywheelConfigOuterLeft;
  private TalonFX shooterIntakeMotor;
  public ModuleConfigurator intakeWheelConfig;
  private TalonFX HoodWheelMotorLeft;
  public ModuleConfigurator HoodMConfigLeft;
  private TalonFX HoodWheelMotorRight;
  public ModuleConfigurator HoodMConfigRight;

  // Defines tunable values , particularly for configurations of motors ( IE PIDs
  // )
  private VelocityTorqueCurrentFOC velIntakeRequest = new VelocityTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC velShooterLeftRequest = new VelocityTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC velShooterRightRequest = new VelocityTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC velShooterOuterRightRequest = new VelocityTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC velShooterOuterLeftRequest = new VelocityTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC velHoodLeftRequest = new VelocityTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC velHoodRightRequest = new VelocityTorqueCurrentFOC(0);

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

  private StatusSignal<AngularVelocity> velocityOfHoodWheelMotorLeftRPS;
  private StatusSignal<Current> statorCurrentOfHoodLeftAmps;
  private StatusSignal<Voltage> outputOfHoodLeftVolts;
  private StatusSignal<AngularAcceleration> accelerationOfHoodLeft;
  private StatusSignal<AngularVelocity> velocityOfHoodWheelMotorRightRPS;
  private StatusSignal<Current> statorCurrentOfHoodRightAmps;
  private StatusSignal<Voltage> outputOfHoodRightVolts;
  private StatusSignal<AngularAcceleration> accelerationOfHoodRight;

  private StatusSignal<AngularVelocity> velocityOfMainFlywheelOuterRightRPS;
  private StatusSignal<Current> statorCurrentOfMainFlywheelOuterRightAmps;
  private StatusSignal<Voltage> outputOfMainFlywheelOuterRightVolts;
  private StatusSignal<AngularAcceleration> accelerationOfMainFlywheelOuterRight;

  private StatusSignal<AngularVelocity> velocityOfMainFlywheelOuterLeftRPS;
  private StatusSignal<Current> statorCurrentOfMainFlywheelOuterLeftAmps;
  private StatusSignal<Voltage> outputOfMainFlywheelOuterLeftVolts;
  private StatusSignal<AngularAcceleration> accelerationOfMainFlywheelOuterLeft;



  public double mainFlywheelSetpoint = 0;
  public double intakeSetpoint = 0;
  public double HoodSetpointRight = 0;
  public double HoodSetpointLeft = 0;

  private TunablePID flywheelLeftPID;
  private TunablePID flywheelRighPID;
  private TunablePID flywheelOuterRightPID;
  private TunablePID flywheelOuterLeftPID;
  private TunablePID intakePID;
  private TunablePID HoodLeftPID;
  private TunablePID HoodRightPID;

  public ShooterRealQuad() {
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
    Gains HoodLeftGains = new Gains.Builder()
        .kP(Constants.ShooterConstants.Left.kHoodMotorkP)
        .kI(Constants.ShooterConstants.Left.kHoodMotorkI)
        .kD(Constants.ShooterConstants.Left.kHoodMotorkD)
        .kS(Constants.ShooterConstants.Left.kHoodMotorkS)
        .kV(Constants.ShooterConstants.Left.kHoodMotorkV)
        .kA(Constants.ShooterConstants.Left.kHoodMotorkA).build();
    Gains HoodRightGains = new Gains.Builder()
        .kP(Constants.ShooterConstants.Right.kHoodMotorkP)
        .kI(Constants.ShooterConstants.Right.kHoodMotorkI)
        .kD(Constants.ShooterConstants.Right.kHoodMotorkD)
        .kS(Constants.ShooterConstants.Right.kHoodMotorkS)
        .kV(Constants.ShooterConstants.Right.kHoodMotorkV)
        .kA(Constants.ShooterConstants.Right.kHoodMotorkA).build();

    setupLeftFlywheel(flywheelGains);
    setupRightFlywheel(flywheelGains);
    setupOuterRightFlywheel(flywheelGains);
    setupOuterLeftFlywheel(flywheelGains);
    setupIntake(intakeGains);
    setupLeftHood(HoodLeftGains);
    setupRightHood(HoodRightGains);
  }

  public void setupLeftFlywheel(Gains g) {
    flywheelLeftPID = new TunablePID(
        "/Shooter/Flywheel/Left/PID", g);
    flywheelConfigLeft = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.SharedFlywheel.FlywheelInnerIDLeft,
        Constants.ShooterConstants.SharedFlywheel.isInvertedInnerLeft,
        Constants.ShooterConstants.SharedFlywheel.isCoastRight,
        Constants.ShooterConstants.SharedFlywheel.statorCurrentLimit,
        Constants.ShooterConstants.SharedFlywheel.supplyCurrentLimit);
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
        Constants.ShooterConstants.SharedFlywheel.isInvertedInnerRight,
        Constants.ShooterConstants.SharedFlywheel.isCoastRight,
        Constants.ShooterConstants.SharedFlywheel.statorCurrentLimit,
        Constants.ShooterConstants.SharedFlywheel.supplyCurrentLimit);
    shooterFlywheelInnerRight = new TalonFX(flywheelConfigRight.getMotorInnerId(), new CANBus("rio"));
    flywheelConfigRight.configureMotor(shooterFlywheelInnerRight, flywheelRighPID);
    velocityOfMainFlywheelRightRPS = shooterFlywheelInnerRight.getVelocity();
    statorCurrentOfMainFlywheelRightAmps = shooterFlywheelInnerRight.getStatorCurrent();
    outputOfMainFlywheelRightVolts = shooterFlywheelInnerRight.getMotorVoltage();
    accelerationOfMainFlywheelRight = shooterFlywheelInnerRight.getAcceleration();
    flywheelConfigRight.configureSignals(shooterFlywheelInnerRight, 50.0, velocityOfMainFlywheelRightRPS,
        statorCurrentOfMainFlywheelRightAmps, outputOfMainFlywheelRightVolts, accelerationOfMainFlywheelRight);
  }

  public void setupOuterRightFlywheel(Gains g) {
    flywheelOuterRightPID = new TunablePID(
        "/Shooter/Flywheel/Outer/PID", g);
    // Flywheel Configuration
    flywheelConfigOuterRight = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.SharedFlywheel.FlywheelOuterIDRight,
        Constants.ShooterConstants.SharedFlywheel.isInvertedOuterRight,
        Constants.ShooterConstants.SharedFlywheel.isCoastRight,
        Constants.ShooterConstants.SharedFlywheel.statorCurrentLimit,
        Constants.ShooterConstants.SharedFlywheel.supplyCurrentLimit);
    shooterFlywheelOuterRight = new TalonFX(flywheelConfigOuterRight.getMotorInnerId(), new CANBus("rio"));
    flywheelConfigOuterRight.configureMotor(shooterFlywheelOuterRight, flywheelOuterRightPID);
    velocityOfMainFlywheelOuterRightRPS = shooterFlywheelInnerRight.getVelocity();
    statorCurrentOfMainFlywheelOuterRightAmps = shooterFlywheelInnerRight.getStatorCurrent();
    outputOfMainFlywheelOuterRightVolts = shooterFlywheelInnerRight.getMotorVoltage();
    accelerationOfMainFlywheelOuterRight = shooterFlywheelInnerRight.getAcceleration();
    flywheelConfigOuterRight.configureSignals(shooterFlywheelOuterRight, 50.0, velocityOfMainFlywheelOuterRightRPS,
        statorCurrentOfMainFlywheelOuterRightAmps, outputOfMainFlywheelOuterRightVolts,
        accelerationOfMainFlywheelOuterRight);
  }

    public void setupOuterLeftFlywheel(Gains g) {
    flywheelOuterLeftPID = new TunablePID(
        "/Shooter/Flywheel/Outer/PID", g);
    // Flywheel Configuration
    flywheelConfigOuterLeft = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.SharedFlywheel.FlywheelOuterIDLeft,
        Constants.ShooterConstants.SharedFlywheel.isInvertedOuterLeft,
        Constants.ShooterConstants.SharedFlywheel.isCoastLeft,
        Constants.ShooterConstants.SharedFlywheel.statorCurrentLimit,
        Constants.ShooterConstants.SharedFlywheel.supplyCurrentLimit);
    shooterFlywheelOuterLeft = new TalonFX(flywheelConfigOuterLeft.getMotorInnerId(), new CANBus("rio"));
    flywheelConfigOuterLeft.configureMotor(shooterFlywheelOuterLeft, flywheelOuterLeftPID);
    velocityOfMainFlywheelOuterLeftRPS = shooterFlywheelOuterLeft.getVelocity();
    statorCurrentOfMainFlywheelOuterLeftAmps = shooterFlywheelOuterLeft.getStatorCurrent();
    outputOfMainFlywheelOuterLeftVolts = shooterFlywheelOuterLeft.getMotorVoltage();
    accelerationOfMainFlywheelOuterLeft = shooterFlywheelOuterLeft.getAcceleration();
    flywheelConfigOuterLeft.configureSignals(shooterFlywheelOuterLeft, 50.0, velocityOfMainFlywheelOuterLeftRPS,
        statorCurrentOfMainFlywheelOuterLeftAmps, outputOfMainFlywheelOuterLeftVolts,
        accelerationOfMainFlywheelOuterLeft);
  }

  public void setupIntake(Gains g) {
    // Intake Configuration
    intakePID = new TunablePID(
        "/Shooter/Intake/PID", g);
    intakeWheelConfig = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.SharedIntake.intakeIDLeft,
        Constants.ShooterConstants.SharedIntake.isInverted,
        Constants.ShooterConstants.SharedIntake.isCoast,
        Constants.ShooterConstants.SharedIntake.statorCurrentLimit,
        Constants.ShooterConstants.SharedIntake.supplyCurrentLimit);
    shooterIntakeMotor = new TalonFX(intakeWheelConfig.getMotorInnerId(), new CANBus("rio"));
    intakeWheelConfig.configureMotor(shooterIntakeMotor, intakePID);
    velocityOfIntakeRPS = shooterIntakeMotor.getVelocity();
    statorCurrentOfIntakeAmps = shooterIntakeMotor.getStatorCurrent();
    outputOfIntakeVolts = shooterIntakeMotor.getMotorVoltage();
    accelerationOfIntake = shooterIntakeMotor.getAcceleration();
    intakeWheelConfig.configureSignals(shooterIntakeMotor, 50.0, velocityOfIntakeRPS,
        statorCurrentOfIntakeAmps, outputOfIntakeVolts, accelerationOfIntake);
  }

  public void setupLeftHood(Gains g) {
    HoodLeftPID = new TunablePID(
        "/Shooter/Hood/Left/PID", g);
    // Flywheel Configuration
    HoodMConfigLeft = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.Left.HoodID,
        Constants.ShooterConstants.Left.isInverted,
        Constants.ShooterConstants.Left.isCoast,
        Constants.ShooterConstants.Left.statorCurrentLimit,
        Constants.ShooterConstants.Left.supplyCurrentLimit);
    HoodWheelMotorLeft = new TalonFX(HoodMConfigLeft.getMotorInnerId(), new CANBus("rio"));
    HoodMConfigLeft.configureMotor(HoodWheelMotorLeft, HoodLeftPID);
    velocityOfHoodWheelMotorLeftRPS = HoodWheelMotorLeft.getVelocity();
    statorCurrentOfHoodLeftAmps = HoodWheelMotorLeft.getStatorCurrent();
    outputOfHoodLeftVolts = HoodWheelMotorLeft.getMotorVoltage();
    accelerationOfHoodLeft = HoodWheelMotorLeft.getAcceleration();
    HoodMConfigLeft.configureSignals(HoodWheelMotorLeft, 50.0, velocityOfHoodWheelMotorLeftRPS,
        statorCurrentOfHoodLeftAmps, outputOfHoodLeftVolts, accelerationOfHoodLeft);
  }

  public void setupRightHood(Gains g) {
    HoodRightPID = new TunablePID(
        "/Shooter/Hood/Right/PID", g);
    // Flywheel Configuration
    HoodMConfigRight = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.Right.HoodID,
        Constants.ShooterConstants.Right.isInverted,
        Constants.ShooterConstants.Right.isCoast,
        Constants.ShooterConstants.Right.statorCurrentLimit,
        Constants.ShooterConstants.Right.supplyCurrentLimit);
    HoodWheelMotorRight = new TalonFX(HoodMConfigRight.getMotorInnerId(), new CANBus("rio"));
    HoodMConfigRight.configureMotor(HoodWheelMotorRight, HoodRightPID);
    velocityOfHoodWheelMotorRightRPS = HoodWheelMotorRight.getVelocity();
    statorCurrentOfHoodRightAmps = HoodWheelMotorRight.getStatorCurrent();
    outputOfHoodRightVolts = HoodWheelMotorRight.getMotorVoltage();
    accelerationOfHoodRight = HoodWheelMotorRight.getAcceleration();
    flywheelConfigLeft.configureSignals(HoodWheelMotorRight, 50.0, velocityOfHoodWheelMotorRightRPS,
        statorCurrentOfHoodRightAmps, outputOfHoodRightVolts, accelerationOfHoodRight);
  }

  public void updateInputs(ShooterIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        velocityOfMainFlywhelLeftRPS,
        velocityOfMainFlywheelRightRPS,
        velocityOfHoodWheelMotorLeftRPS,
        velocityOfHoodWheelMotorRightRPS,
        velocityOfMainFlywheelOuterRightRPS,
        velocityOfMainFlywheelOuterLeftRPS,
        velocityOfIntakeRPS,
        accelerationOfMainFlywheelLeft,
        accelerationOfMainFlywheelRight,
        accelerationOfMainFlywheelOuterRight,
        accelerationOfMainFlywheelOuterLeft,
        accelerationOfHoodLeft,
        accelerationOfHoodRight,
        accelerationOfIntake,
        statorCurrentOfHoodLeftAmps,
        statorCurrentOfHoodRightAmps,
        statorCurrentOfMainFlywheelLeftAmps,
        statorCurrentOfMainFlywheelRightAmps,
        statorCurrentOfMainFlywheelOuterRightAmps,
        statorCurrentOfMainFlywheelOuterLeftAmps,
        outputOfHoodLeftVolts,
        outputOfHoodRightVolts,
        outputOfMainFlywheelLeftVolts,
        outputOfMainFlywheelRightVolts,
        outputOfMainFlywheelOuterRightVolts,
        outputOfMainFlywheelOuterLeftVolts,
        outputOfIntakeVolts);

    inputs.velocityOfMainFlywheelLeftRPS = velocityOfMainFlywhelLeftRPS.getValue().in(Rotations.per(Seconds));
    inputs.velocityOfMainFlywheelRightRPS = velocityOfMainFlywheelRightRPS.getValue().in(Rotations.per(Seconds));
    inputs.velocityOfMainFlywheelOuterRightRPS = velocityOfMainFlywheelOuterRightRPS.getValue()
        .in(Rotations.per(Seconds));
    inputs.velocityOfMainFlywheelOuterLeftRPS = velocityOfMainFlywheelOuterLeftRPS.getValue()
        .in(Rotations.per(Seconds));
    inputs.velocityOfHoodWheelMotorLeftRPS = velocityOfHoodWheelMotorLeftRPS.getValue()
        .in(Rotations.per(Seconds));
    inputs.velocityOfHoodWheelMotorRightRPS = velocityOfHoodWheelMotorRightRPS.getValue()
        .in(Rotations.per(Seconds));
    inputs.velocityOfIntakeRPS = velocityOfIntakeRPS.getValue().in(Rotations.per(Seconds));
    inputs.accelerationOfMainFlywheelLeft = accelerationOfMainFlywheelLeft.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.accelerationOfMainFlywheelRight = accelerationOfMainFlywheelRight.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.accelerationOfHoodLeft = accelerationOfHoodLeft.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.accelerationOfIntake = accelerationOfIntake.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.statorCurrentOfHoodLeftAmps = statorCurrentOfHoodLeftAmps.getValue().in(Amps);
    inputs.statorCurrentOfHoodRightAmps = statorCurrentOfHoodRightAmps.getValue().in(Amps);
    inputs.statorCurrentOfMainFlywheelLeftAmps = statorCurrentOfMainFlywheelLeftAmps.getValue().in(Amps);
    inputs.statorCurrentOfMainFlywheelRightAmps = statorCurrentOfMainFlywheelRightAmps.getValue().in(Amps);
    inputs.statorCurrentOfMainFlywheelOuterRightAmps = statorCurrentOfMainFlywheelOuterRightAmps.getValue().in(Amps);
    inputs.statorCurrentOfMainFlywheelOuterLeftAmps = statorCurrentOfMainFlywheelOuterLeftAmps.getValue().in(Amps);
    inputs.statorCurrentOfIntakeAmps = statorCurrentOfIntakeAmps.getValue().in(Amps);

    inputs.HoodWheelMotorRightConnected = HoodWheelMotorRight.isConnected();
    inputs.HoodWheelMotorLeftConnected = HoodWheelMotorLeft.isConnected();
    inputs.shooterFlywheelInnerLeftConnected = shooterFlywheelInnerLeft.isConnected();
    inputs.shooterFlywheelInnerRightConnected = shooterFlywheelInnerRight.isConnected();
    inputs.shooterFlywheelOuterRightConnected = shooterFlywheelOuterRight.isConnected();
    inputs.shooterFlywheelOuterLeftConnected = shooterFlywheelOuterLeft.isConnected();
    inputs.shooterIntakeMotorConnected = shooterIntakeMotor.isConnected();

    inputs.outputOfHoodLeftVolts = outputOfHoodLeftVolts.getValue().in(Volts);
    inputs.outputOfHoodRightVolts = outputOfHoodRightVolts.getValue().in(Volts);
    inputs.outputOfMainFlywheelLeftVolts = outputOfMainFlywheelLeftVolts.getValue().in(Volts);
    inputs.outputOfMainFlywheelRightVolts = outputOfMainFlywheelRightVolts.getValue().in(Volts);
    inputs.outputOfMainFlywheelOuterRightVolts = outputOfMainFlywheelOuterRightVolts.getValue().in(Volts);
    inputs.outputOfMainFlywheelOuterRightVolts = outputOfMainFlywheelOuterLeftVolts.getValue().in(Volts);
    inputs.outputOfIntakeVolts = outputOfIntakeVolts.getValue().in(Volts);

  }

  public void setOutput(double shooterOutput, double HoodOutputLeft, double HoodOutputRight) {
    shooterFlywheelInnerLeft.set(shooterOutput);
    shooterFlywheelInnerRight.set(shooterOutput);
    shooterFlywheelOuterRight.set(shooterOutput);
    shooterFlywheelOuterLeft.set(shooterOutput);
    HoodWheelMotorLeft.set(HoodOutputLeft);
    HoodWheelMotorRight.set(HoodOutputRight);
  }

  public void setVelocity(ShooterState desiredState) {
    setVelocity(desiredState.getFlywheelSpeed(),
        desiredState.getHoodSpeed(),
        desiredState.getHoodSpeed(), desiredState.getIntakeSpeed());
  }

  public void setVelocity(double shooterFlywheelSpeed, double shooterHoodSpeedOfLeft,
      double shooterHoodSpeedOfRight, double shooterIntakeSpeed) {
    setMainWheelSpeed(shooterFlywheelSpeed);
    setHoodSpeedOfLeft(shooterHoodSpeedOfLeft);
    setHoodSpeedOfRight(shooterHoodSpeedOfRight);
    setIntakeSpeed(shooterIntakeSpeed);
  }

  public void setMainWheelSpeed(double shooterFlywheelSpeedInRPS) {
    mainFlywheelSetpoint = shooterFlywheelSpeedInRPS;
    shooterFlywheelInnerLeft.setControl(velShooterLeftRequest.withVelocity(mainFlywheelSetpoint));
    shooterFlywheelInnerRight.setControl(velShooterRightRequest.withVelocity(mainFlywheelSetpoint));
    shooterFlywheelOuterRight.setControl(velShooterOuterRightRequest.withVelocity(mainFlywheelSetpoint));
    shooterFlywheelOuterLeft.setControl(velShooterOuterLeftRequest.withVelocity(mainFlywheelSetpoint));
  }

  public void setHoodSpeedOfLeft(double shooterHoodSpeedInRPS) {
    HoodSetpointLeft = shooterHoodSpeedInRPS;
    Logger.recordOutput("/Shooter/Hood/LeftSetPointSpeed",HoodSetpointLeft);
    HoodWheelMotorLeft.setControl(velHoodLeftRequest.withVelocity(HoodSetpointLeft));
  }

  public void setHoodSpeedOfRight(double shooterHoodSpeedInRPS) {
    HoodSetpointRight = shooterHoodSpeedInRPS;
    Logger.recordOutput("/Shooter/Hood/RightSetPointSpeed",HoodSetpointRight);
    HoodWheelMotorRight.setControl(velHoodRightRequest.withVelocity(HoodSetpointRight));
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
    shooterFlywheelOuterRight.stopMotor();
  }

  public void stopHoodLeftWheel() {
    HoodSetpointLeft = 0;
    
    Logger.recordOutput("/Shooter/Hood/LeftSetPointSpeed",HoodSetpointLeft);
    HoodWheelMotorLeft.stopMotor();
  }

  public void stopHoodRightWheel() {
    HoodSetpointRight = 0;
    
    Logger.recordOutput("/Shooter/Hood/RightSetPointSpeed",HoodSetpointRight);
    HoodWheelMotorRight.stopMotor();
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
    if (flywheelOuterRightPID.hasChanged()) {
      flywheelConfigOuterRight.updateMotorPID(shooterFlywheelOuterRight, flywheelOuterRightPID);
    }
    if (flywheelOuterLeftPID.hasChanged()) {
      flywheelConfigOuterLeft.updateMotorPID(shooterFlywheelOuterLeft, flywheelOuterLeftPID);
    }

    if (HoodLeftPID.hasChanged()) {
      HoodMConfigLeft.updateMotorPID(HoodWheelMotorRight, HoodLeftPID);
    }
    if (HoodRightPID.hasChanged()) {
      HoodMConfigRight.updateMotorPID(HoodWheelMotorRight, HoodRightPID);
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
    shooterFlywheelOuterRight.setControl(switch (ClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
    shooterFlywheelOuterLeft.setControl(switch (ClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity_Flywheel() {
    double avg = (shooterFlywheelInnerLeft.getVelocity().getValue().in(RotationsPerSecond)
                + shooterFlywheelInnerRight.getVelocity().getValue().in(RotationsPerSecond)
                + shooterFlywheelOuterLeft.getVelocity().getValue().in(RotationsPerSecond)
                + shooterFlywheelOuterRight.getVelocity().getValue().in(RotationsPerSecond)) / 4;
    return avg;
  }

  /* Characterization */
  public void runCharacterization_Hood(double output) {
    HoodWheelMotorLeft.setControl(switch (CharacterizationClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
    HoodWheelMotorRight.setControl(switch (CharacterizationClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity_Hood() {
    double avg = (HoodWheelMotorLeft.getVelocity().getValue().in(RotationsPerSecond) +
        HoodWheelMotorRight.getVelocity().getValue().in(RotationsPerSecond)) / 2;
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