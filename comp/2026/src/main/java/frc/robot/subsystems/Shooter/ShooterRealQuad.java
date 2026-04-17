package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.security.spec.ECPublicKeySpec;

import org.bobcatrobotics.Hardware.Characterization.CharacterizationClosedLoopOutputType;
import org.bobcatrobotics.Hardware.Motors.LoggedStallDetector;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.RobotState;
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

  LoggedStallDetector stallHoodLeftDetector;
  LoggedStallDetector stallHoodRightDetector;
  LoggedStallDetector stallInnerLeftDetector;
  LoggedStallDetector stallInnerRightDetector;
  LoggedStallDetector stallOuterLeftDetector;
  LoggedStallDetector stallOuterRightDetector;

  public ShooterRealQuad() {
    // Flywheel Configuration
    Gains flywheelGains = new Gains.Builder()
        .kP(Constants.ShooterConstants.SharedFlywheel.kshooterMainkP)
        .kI(Constants.ShooterConstants.SharedFlywheel.kshooterMainkI)
        .kD(Constants.ShooterConstants.SharedFlywheel.kshooterMainkD)
        .kS(Constants.ShooterConstants.SharedFlywheel.kshooterMainkS)
        .kV(Constants.ShooterConstants.SharedFlywheel.kshooterMainkV)
        .kA(Constants.ShooterConstants.SharedFlywheel.kshooterMainkA).build();
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
    setupLeftHood(HoodLeftGains);
    setupRightHood(HoodRightGains);

    stallHoodLeftDetector = new LoggedStallDetector(
        "Shooter/Hood/Left", // unique name per motor
        1.0, // velocity threshold
        40.0, // current threshold
        0.25 // time threshold
    );
    stallHoodRightDetector = new LoggedStallDetector(
        "Shooter/Hood/Right", // unique name per motor
        1.0, // velocity threshold
        40.0, // current threshold
        0.25 // time threshold
    );
    stallInnerLeftDetector = new LoggedStallDetector(
        "Shooter/Inner/Left", // unique name per motor
        1.0, // velocity threshold
        40.0, // current threshold
        0.25 // time threshold
    );
    stallInnerRightDetector = new LoggedStallDetector(
        "Shooter/Inner/Right", // unique name per motor
        1.0, // velocity threshold
        40.0, // current threshold
        0.25 // time threshold
    );
    stallOuterLeftDetector = new LoggedStallDetector(
        "Shooter/Outer/Left", // unique name per motor
        1.0, // velocity threshold
        40.0, // current threshold
        0.25 // time threshold
    );
    stallOuterRightDetector = new LoggedStallDetector(
        "Shooter/Outer/Right", // unique name per motor
        1.0, // velocity threshold
        40.0, // current threshold
        0.25 // time threshold
    );
  }

  public void setupLeftFlywheel(Gains g) {
    flywheelConfigLeft = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.SharedFlywheel.FlywheelInnerIDLeft,
        Constants.ShooterConstants.SharedFlywheel.isInvertedInnerLeft,
        Constants.ShooterConstants.SharedFlywheel.isCoastRight,
        Constants.ShooterConstants.SharedFlywheel.statorCurrentLimit,
        Constants.ShooterConstants.SharedFlywheel.supplyCurrentLimit);
    shooterFlywheelInnerLeft = new TalonFX(flywheelConfigLeft.getMotorInnerId(), new CANBus("rio"));
    flywheelConfigLeft.configureMotor(shooterFlywheelInnerLeft, g);
    if (Constants.lowTelemetryMode) {
      velocityOfMainFlywhelLeftRPS = shooterFlywheelInnerLeft.getVelocity();
      statorCurrentOfMainFlywheelLeftAmps = shooterFlywheelInnerLeft.getStatorCurrent();
      flywheelConfigLeft.configureSignals(shooterFlywheelInnerLeft, 50.0, velocityOfMainFlywhelLeftRPS,
          statorCurrentOfMainFlywheelLeftAmps);
    } else {
      velocityOfMainFlywhelLeftRPS = shooterFlywheelInnerLeft.getVelocity();
      statorCurrentOfMainFlywheelLeftAmps = shooterFlywheelInnerLeft.getStatorCurrent();
      outputOfMainFlywheelLeftVolts = shooterFlywheelInnerLeft.getMotorVoltage();
      accelerationOfMainFlywheelLeft = shooterFlywheelInnerLeft.getAcceleration();
      flywheelConfigLeft.configureSignals(shooterFlywheelInnerLeft, 50.0, velocityOfMainFlywhelLeftRPS,
          statorCurrentOfMainFlywheelLeftAmps, outputOfMainFlywheelLeftVolts, accelerationOfMainFlywheelLeft);
    }
  }

  public void setupRightFlywheel(Gains g) {
    // Flywheel Configuration
    flywheelConfigRight = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.SharedFlywheel.FlywheelInnerIDRight,
        Constants.ShooterConstants.SharedFlywheel.isInvertedInnerRight,
        Constants.ShooterConstants.SharedFlywheel.isCoastRight,
        Constants.ShooterConstants.SharedFlywheel.statorCurrentLimit,
        Constants.ShooterConstants.SharedFlywheel.supplyCurrentLimit);
    shooterFlywheelInnerRight = new TalonFX(flywheelConfigRight.getMotorInnerId(), new CANBus("rio"));
    flywheelConfigRight.configureMotor(shooterFlywheelInnerRight, g);
    if (Constants.lowTelemetryMode) {
      velocityOfMainFlywheelRightRPS = shooterFlywheelInnerRight.getVelocity();
      statorCurrentOfMainFlywheelRightAmps = shooterFlywheelInnerRight.getStatorCurrent();
      flywheelConfigRight.configureSignals(shooterFlywheelInnerRight, 50.0, velocityOfMainFlywheelRightRPS,
          statorCurrentOfMainFlywheelRightAmps);
    } else {
      velocityOfMainFlywheelRightRPS = shooterFlywheelInnerRight.getVelocity();
      statorCurrentOfMainFlywheelRightAmps = shooterFlywheelInnerRight.getStatorCurrent();
      outputOfMainFlywheelRightVolts = shooterFlywheelInnerRight.getMotorVoltage();
      accelerationOfMainFlywheelRight = shooterFlywheelInnerRight.getAcceleration();
      flywheelConfigRight.configureSignals(shooterFlywheelInnerRight, 50.0, velocityOfMainFlywheelRightRPS,
          statorCurrentOfMainFlywheelRightAmps, outputOfMainFlywheelRightVolts, accelerationOfMainFlywheelRight);
    }

  }

  public void setupOuterRightFlywheel(Gains g) {
    // Flywheel Configuration
    flywheelConfigOuterRight = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.SharedFlywheel.FlywheelOuterIDRight,
        Constants.ShooterConstants.SharedFlywheel.isInvertedOuterRight,
        Constants.ShooterConstants.SharedFlywheel.isCoastRight,
        Constants.ShooterConstants.SharedFlywheel.statorCurrentLimit,
        Constants.ShooterConstants.SharedFlywheel.supplyCurrentLimit);
    shooterFlywheelOuterRight = new TalonFX(flywheelConfigOuterRight.getMotorInnerId(), new CANBus("rio"));
    flywheelConfigOuterRight.configureMotor(shooterFlywheelOuterRight, g);
    if (Constants.lowTelemetryMode) {
      velocityOfMainFlywheelOuterRightRPS = shooterFlywheelInnerRight.getVelocity();
      statorCurrentOfMainFlywheelOuterRightAmps = shooterFlywheelInnerRight.getStatorCurrent();
      flywheelConfigOuterRight.configureSignals(shooterFlywheelOuterRight, 50.0, velocityOfMainFlywheelOuterRightRPS,
          statorCurrentOfMainFlywheelOuterRightAmps);
    } else {
      velocityOfMainFlywheelOuterRightRPS = shooterFlywheelInnerRight.getVelocity();
      statorCurrentOfMainFlywheelOuterRightAmps = shooterFlywheelInnerRight.getStatorCurrent();
      outputOfMainFlywheelOuterRightVolts = shooterFlywheelInnerRight.getMotorVoltage();
      accelerationOfMainFlywheelOuterRight = shooterFlywheelInnerRight.getAcceleration();
      flywheelConfigOuterRight.configureSignals(shooterFlywheelOuterRight, 50.0, velocityOfMainFlywheelOuterRightRPS,
          statorCurrentOfMainFlywheelOuterRightAmps, outputOfMainFlywheelOuterRightVolts,
          accelerationOfMainFlywheelOuterRight);
    }

  }

  public void setupOuterLeftFlywheel(Gains g) {
    // Flywheel Configuration
    flywheelConfigOuterLeft = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.SharedFlywheel.FlywheelOuterIDLeft,
        Constants.ShooterConstants.SharedFlywheel.isInvertedOuterLeft,
        Constants.ShooterConstants.SharedFlywheel.isCoastLeft,
        Constants.ShooterConstants.SharedFlywheel.statorCurrentLimit,
        Constants.ShooterConstants.SharedFlywheel.supplyCurrentLimit);
    shooterFlywheelOuterLeft = new TalonFX(flywheelConfigOuterLeft.getMotorInnerId(), new CANBus("rio"));
    flywheelConfigOuterLeft.configureMotor(shooterFlywheelOuterLeft, g);
    if (Constants.lowTelemetryMode) {
      velocityOfMainFlywheelOuterLeftRPS = shooterFlywheelOuterLeft.getVelocity();
      statorCurrentOfMainFlywheelOuterLeftAmps = shooterFlywheelOuterLeft.getStatorCurrent();
      flywheelConfigOuterLeft.configureSignals(shooterFlywheelOuterLeft, 50.0, velocityOfMainFlywheelOuterLeftRPS,
          statorCurrentOfMainFlywheelOuterLeftAmps);
    } else {
      velocityOfMainFlywheelOuterLeftRPS = shooterFlywheelOuterLeft.getVelocity();
      statorCurrentOfMainFlywheelOuterLeftAmps = shooterFlywheelOuterLeft.getStatorCurrent();
      outputOfMainFlywheelOuterLeftVolts = shooterFlywheelOuterLeft.getMotorVoltage();
      accelerationOfMainFlywheelOuterLeft = shooterFlywheelOuterLeft.getAcceleration();
      flywheelConfigOuterLeft.configureSignals(shooterFlywheelOuterLeft, 50.0, velocityOfMainFlywheelOuterLeftRPS,
          statorCurrentOfMainFlywheelOuterLeftAmps, outputOfMainFlywheelOuterLeftVolts,
          accelerationOfMainFlywheelOuterLeft);
    }

  }

  public void setupLeftHood(Gains g) {
    // Flywheel Configuration
    HoodMConfigLeft = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.Left.HoodID,
        Constants.ShooterConstants.Left.isInverted,
        Constants.ShooterConstants.Left.isCoast,
        Constants.ShooterConstants.Left.statorCurrentLimit,
        Constants.ShooterConstants.Left.supplyCurrentLimit);
    HoodWheelMotorLeft = new TalonFX(HoodMConfigLeft.getMotorInnerId(), new CANBus("rio"));
    HoodMConfigLeft.configureMotor(HoodWheelMotorLeft, g);
    if (Constants.lowTelemetryMode) {
      velocityOfHoodWheelMotorLeftRPS = HoodWheelMotorLeft.getVelocity();
      statorCurrentOfHoodLeftAmps = HoodWheelMotorLeft.getStatorCurrent();
      HoodMConfigLeft.configureSignals(HoodWheelMotorLeft, 50.0, velocityOfHoodWheelMotorLeftRPS,
          statorCurrentOfHoodLeftAmps);
    } else {
      velocityOfHoodWheelMotorLeftRPS = HoodWheelMotorLeft.getVelocity();
      statorCurrentOfHoodLeftAmps = HoodWheelMotorLeft.getStatorCurrent();
      outputOfHoodLeftVolts = HoodWheelMotorLeft.getMotorVoltage();
      accelerationOfHoodLeft = HoodWheelMotorLeft.getAcceleration();
      HoodMConfigLeft.configureSignals(HoodWheelMotorLeft, 50.0, velocityOfHoodWheelMotorLeftRPS,
          statorCurrentOfHoodLeftAmps, outputOfHoodLeftVolts, accelerationOfHoodLeft);
    }

  }

  public void setupRightHood(Gains g) {
    // Flywheel Configuration
    HoodMConfigRight = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.Right.HoodID,
        Constants.ShooterConstants.Right.isInverted,
        Constants.ShooterConstants.Right.isCoast,
        Constants.ShooterConstants.Right.statorCurrentLimit,
        Constants.ShooterConstants.Right.supplyCurrentLimit);
    HoodWheelMotorRight = new TalonFX(HoodMConfigRight.getMotorInnerId(), new CANBus("rio"));
    HoodMConfigRight.configureMotor(HoodWheelMotorRight, g);
    if (Constants.lowTelemetryMode) {
      velocityOfHoodWheelMotorRightRPS = HoodWheelMotorRight.getVelocity();
      statorCurrentOfHoodRightAmps = HoodWheelMotorRight.getStatorCurrent();
      flywheelConfigLeft.configureSignals(HoodWheelMotorRight, 50.0, velocityOfHoodWheelMotorRightRPS,
          statorCurrentOfHoodRightAmps);
    } else {
      velocityOfHoodWheelMotorRightRPS = HoodWheelMotorRight.getVelocity();
      statorCurrentOfHoodRightAmps = HoodWheelMotorRight.getStatorCurrent();
      outputOfHoodRightVolts = HoodWheelMotorRight.getMotorVoltage();
      accelerationOfHoodRight = HoodWheelMotorRight.getAcceleration();
      flywheelConfigLeft.configureSignals(HoodWheelMotorRight, 50.0, velocityOfHoodWheelMotorRightRPS,
          statorCurrentOfHoodRightAmps, outputOfHoodRightVolts, accelerationOfHoodRight);
    }

  }

  public void updateInputs(ShooterIOInputs inputs) {
    if (Constants.lowTelemetryMode) {
      lowTelemetry(inputs);
    } else {
      highTelemetry(inputs);
    }
  }

  public void highTelemetry(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        accelerationOfMainFlywheelLeft,
        accelerationOfMainFlywheelRight,
        accelerationOfMainFlywheelOuterRight,
        accelerationOfMainFlywheelOuterLeft,
        accelerationOfHoodLeft,
        accelerationOfHoodRight,
        outputOfHoodLeftVolts,
        outputOfHoodRightVolts,
        outputOfMainFlywheelLeftVolts,
        outputOfMainFlywheelRightVolts,
        outputOfMainFlywheelOuterRightVolts,
        outputOfMainFlywheelOuterLeftVolts);

    inputs.accelerationOfMainFlywheelLeft = accelerationOfMainFlywheelLeft.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.accelerationOfMainFlywheelRight = accelerationOfMainFlywheelRight.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.accelerationOfHoodLeft = accelerationOfHoodLeft.getValue()
        .in(RotationsPerSecondPerSecond);

    inputs.outputOfHoodLeftVolts = outputOfHoodLeftVolts.getValue().in(Volts);
    inputs.outputOfHoodRightVolts = outputOfHoodRightVolts.getValue().in(Volts);
    inputs.outputOfMainFlywheelLeftVolts = outputOfMainFlywheelLeftVolts.getValue().in(Volts);
    inputs.outputOfMainFlywheelRightVolts = outputOfMainFlywheelRightVolts.getValue().in(Volts);
    inputs.outputOfMainFlywheelOuterRightVolts = outputOfMainFlywheelOuterRightVolts.getValue().in(Volts);
    inputs.outputOfMainFlywheelOuterRightVolts = outputOfMainFlywheelOuterLeftVolts.getValue().in(Volts);
    lowTelemetry(inputs);
  }

  public void lowTelemetry(ShooterIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        velocityOfMainFlywhelLeftRPS,
        velocityOfMainFlywheelRightRPS,
        velocityOfHoodWheelMotorLeftRPS,
        velocityOfHoodWheelMotorRightRPS,
        velocityOfMainFlywheelOuterRightRPS,
        velocityOfMainFlywheelOuterLeftRPS,
        statorCurrentOfHoodLeftAmps,
        statorCurrentOfHoodRightAmps,
        statorCurrentOfMainFlywheelLeftAmps,
        statorCurrentOfMainFlywheelRightAmps,
        statorCurrentOfMainFlywheelOuterRightAmps,
        statorCurrentOfMainFlywheelOuterLeftAmps);

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

    inputs.statorCurrentOfHoodLeftAmps = statorCurrentOfHoodLeftAmps.getValue().in(Amps);
    inputs.statorCurrentOfHoodRightAmps = statorCurrentOfHoodRightAmps.getValue().in(Amps);
    inputs.statorCurrentOfMainFlywheelLeftAmps = statorCurrentOfMainFlywheelLeftAmps.getValue().in(Amps);
    inputs.statorCurrentOfMainFlywheelRightAmps = statorCurrentOfMainFlywheelRightAmps.getValue().in(Amps);
    inputs.statorCurrentOfMainFlywheelOuterRightAmps = statorCurrentOfMainFlywheelOuterRightAmps.getValue().in(Amps);
    inputs.statorCurrentOfMainFlywheelOuterLeftAmps = statorCurrentOfMainFlywheelOuterLeftAmps.getValue().in(Amps);

    inputs.HoodWheelMotorRightConnected = HoodWheelMotorRight.isConnected();
    inputs.HoodWheelMotorLeftConnected = HoodWheelMotorLeft.isConnected();
    inputs.shooterFlywheelInnerLeftConnected = shooterFlywheelInnerLeft.isConnected();
    inputs.shooterFlywheelInnerRightConnected = shooterFlywheelInnerRight.isConnected();
    inputs.shooterFlywheelOuterRightConnected = shooterFlywheelOuterRight.isConnected();
    inputs.shooterFlywheelOuterLeftConnected = shooterFlywheelOuterLeft.isConnected();

    boolean isHoodRightDriving = Math.abs(RobotState.getInstance().getCarwashState().getIntakeSpeed()) > 5;
    inputs.HoodWheelMotorRightStalled = stallHoodLeftDetector.update(
        isHoodRightDriving ? inputs.velocityOfHoodWheelMotorRightRPS : Double.MAX_VALUE,
        inputs.torqueCurrentOfHoodLeftAmps);
    boolean isHoodLeftDriving = Math.abs(RobotState.getInstance().getCarwashState().getIntakeSpeed()) > 5;
    inputs.HoodWheelMotorLeftStalled = stallHoodRightDetector.update(
        isHoodLeftDriving ? inputs.velocityOfHoodWheelMotorLeftRPS : Double.MAX_VALUE,
        inputs.torqueCurrentOfHoodRightAmps);
    boolean isInnerLeftDriving = Math.abs(RobotState.getInstance().getCarwashState().getIntakeSpeed()) > 5;
    inputs.shooterFlywheelInnerLeftStalled = stallInnerLeftDetector.update(
        isInnerLeftDriving ? inputs.velocityOfMainFlywheelLeftRPS : Double.MAX_VALUE,
        inputs.torqueCurrentOfMainFlywheelLeftAmps);
    boolean isInnerRightDriving = Math.abs(RobotState.getInstance().getCarwashState().getIntakeSpeed()) > 5;
    inputs.shooterFlywheelInnerRightStalled = stallInnerRightDetector.update(
        isInnerRightDriving ? inputs.velocityOfMainFlywheelRightRPS : Double.MAX_VALUE,
        inputs.torqueCurrentOfMainFlywheelRightAmps);
    boolean isOuterLeftDriving = Math.abs(RobotState.getInstance().getCarwashState().getIntakeSpeed()) > 5;
    inputs.shooterFlywheelOuterLeftStalled = stallOuterLeftDetector.update(
        isOuterLeftDriving ? inputs.velocityOfHoodWheelMotorLeftRPS : Double.MAX_VALUE,
        inputs.torqueCurrentOfMainFlywheelOuterLeftAmps);
    boolean isOuterRightDriving = Math.abs(RobotState.getInstance().getCarwashState().getIntakeSpeed()) > 5;
    inputs.shooterFlywheelOuterRightStalled = stallOuterRightDetector.update(
        isOuterRightDriving ? inputs.velocityOfMainFlywheelOuterRightRPS : Double.MAX_VALUE,
        inputs.torqueCurrentOfMainFlywheelOuterRightAmps);

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
        desiredState.getHoodSpeed());
  }

  public void setVelocity(double shooterFlywheelSpeed, double shooterHoodSpeedOfLeft,
      double shooterHoodSpeedOfRight) {
    setMainWheelSpeed(shooterFlywheelSpeed);
    setHoodSpeedOfLeft(shooterHoodSpeedOfLeft);
    setHoodSpeedOfRight(shooterHoodSpeedOfRight);
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
    Logger.recordOutput("/Shooter/Hood/LeftSetPointSpeed", HoodSetpointLeft);
    HoodWheelMotorLeft.setControl(velHoodLeftRequest.withVelocity(HoodSetpointLeft));
  }

  public void setHoodSpeedOfRight(double shooterHoodSpeedInRPS) {
    HoodSetpointRight = shooterHoodSpeedInRPS;
    Logger.recordOutput("/Shooter/Hood/RightSetPointSpeed", HoodSetpointRight);
    HoodWheelMotorRight.setControl(velHoodRightRequest.withVelocity(HoodSetpointRight));
  }

  public void holdPosition() {
  }

  @Override
  public void stop() {
    stopMainWheel();
    stopHoodLeftWheel();
    stopHoodRightWheel();
    setVelocity(0, 0, 0);
  }

  public void stopMainWheel() {
    mainFlywheelSetpoint = 0;
    shooterFlywheelInnerLeft.stopMotor();
    shooterFlywheelInnerRight.stopMotor();
    shooterFlywheelOuterRight.stopMotor();
  }

  public void stopHoodLeftWheel() {
    HoodSetpointLeft = 0;

    Logger.recordOutput("/Shooter/Hood/LeftSetPointSpeed", HoodSetpointLeft);
    HoodWheelMotorLeft.stopMotor();
  }

  public void stopHoodRightWheel() {
    HoodSetpointRight = 0;

    Logger.recordOutput("/Shooter/Hood/RightSetPointSpeed", HoodSetpointRight);
    HoodWheelMotorRight.stopMotor();
  }

  @Override
  public void periodic() {
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

}