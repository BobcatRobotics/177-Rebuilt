package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.security.spec.ECPublicKeySpec;

import org.bobcatrobotics.Hardware.Characterization.CharacterizationClosedLoopOutputType;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
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
import frc.robot.subsystems.Shooter.Modules.ModuleConfigurator;
import org.bobcatrobotics.Util.Tunables.Gains;
import org.bobcatrobotics.Util.Tunables.TunablePID;

public class ShooterRealDrum implements ShooterIO {
  private TalonFX dumperLeftUp;
  public ModuleConfigurator dumperLeftUpConfig;
  private TalonFX dumperLeftDown;
  public ModuleConfigurator dumperLeftDownConfig;
  private TalonFX dumperRightUp;
  public ModuleConfigurator dumperRightUpConfig;
  private TalonFX dumperRightDown;
  public ModuleConfigurator dumperRightDownConfig;
  // private TalonFX HoodWheelMotorLeft;
  // public ModuleConfigurator HoodMConfigLeft;
  // private TalonFX HoodWheelMotorRight;
  // public ModuleConfigurator HoodMConfigRight;
  private TalonFX adjustableHood;
  private ModuleConfigurator adjustableHoodConfigurator;

  // Defines tunable values , particularly for configurations of motors ( IE PIDs
  // )
  private VelocityTorqueCurrentFOC velDumperLeftUpRequest = new VelocityTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC velDumperLeftDownRequest = new VelocityTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC velDumperRightUpRequest = new VelocityTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC velDumperRightDownRequest = new VelocityTorqueCurrentFOC(0);
  // private VelocityTorqueCurrentFOC velHoodLeftRequest = new VelocityTorqueCurrentFOC(0);
  // private VelocityTorqueCurrentFOC velHoodRightRequest = new VelocityTorqueCurrentFOC(0);
  private PositionTorqueCurrentFOC posAdjustableHoodRequest = new PositionTorqueCurrentFOC(0);
  

  private TorqueCurrentFOC characterizationRequestTorqueCurrentFOC = new TorqueCurrentFOC(0);
  private VoltageOut characterizationRequestVoltage = new VoltageOut(0);

  private StatusSignal<AngularVelocity> velocityOfDumperLeftUpRPS;
  private StatusSignal<Current> statorCurrentOfOfDumperLeftUpAmps;
  private StatusSignal<Voltage> outputOfDumperLeftUpVolts;
  private StatusSignal<AngularAcceleration> accelerationOfDumperLeftUp;

  private StatusSignal<AngularVelocity> velocityOfDumperLeftDownRPS;
  private StatusSignal<Current> statorCurrentOfOfDumperLeftDownAmps;
  private StatusSignal<Voltage> outputOfDumperLeftDownVolts;
  private StatusSignal<AngularAcceleration> accelerationOfDumperLeftDown;
  
  // private StatusSignal<AngularVelocity> velocityOfHoodWheelMotorLeftRPS;
  // private StatusSignal<Current> statorCurrentOfHoodLeftAmps;
  // private StatusSignal<Voltage> outputOfHoodLeftVolts;
  // private StatusSignal<AngularAcceleration> accelerationOfHoodLeft;

  // private StatusSignal<AngularVelocity> velocityOfHoodWheelMotorRightRPS;
  // private StatusSignal<Current> statorCurrentOfHoodRightAmps;
  // private StatusSignal<Voltage> outputOfHoodRightVolts;
  // private StatusSignal<AngularAcceleration> accelerationOfHoodRight;

  private StatusSignal<AngularVelocity> velocityOfDumperRightUpRPS;
  private StatusSignal<Current> statorCurrentOfOfDumperRightUpAmps;
  private StatusSignal<Voltage> outputOfDumperRightUpVolts;
  private StatusSignal<AngularAcceleration> accelerationOfDumperRightUp;

  private StatusSignal<AngularVelocity> velocityOfDumperRightDownRPS;
  private StatusSignal<Current> statorCurrentOfOfDumperRightDownAmps;
  private StatusSignal<Voltage> outputOfDumperRightDownVolts;
  private StatusSignal<AngularAcceleration> accelerationOfDumperRightDown;

  private StatusSignal<AngularVelocity> velocityOfAdjustableHoodPositionRPS;
  private StatusSignal<Current> statorCurrentOfAdjustableHoodPositionAmps;
  private StatusSignal<Voltage> outputOfAdjustableHoodPositionVolts;
  private StatusSignal<AngularAcceleration> accelerationOfAdjustableHoodPosition;



  public double dumperLeftUpSetPoint = 0;
  public double dumperLeftDownSetPoint = 0;
  public double dumperRightUpSetPoint = 0;
  public double dumperRightDownSetPoint = 0;
  public double adjustableHoodSetPoint = 0;

  public ShooterRealDrum() {
    // Flywheel Configuration
    Gains dumperLeftUpGains = new Gains.Builder()
        .kP(Constants.ShooterConstants.Left.kdumperLeftMotorkP)
        .kI(Constants.ShooterConstants.Left.kdumperLeftMotorkI)
        .kD(Constants.ShooterConstants.Left.kdumperLeftMotorkD)
        .kS(Constants.ShooterConstants.Left.kdumperLeftMotorkS)
        .kV(Constants.ShooterConstants.Left.kdumperLeftMotorkV)
        .kA(Constants.ShooterConstants.Left.kdumperLeftMotorkA).build();
    Gains dumperLeftDownGains = new Gains.Builder()
        .kP(Constants.ShooterConstants.Left.kdumperLeftMotorkP)
        .kI(Constants.ShooterConstants.Left.kdumperLeftMotorkI)
        .kD(Constants.ShooterConstants.Left.kdumperLeftMotorkD)
        .kS(Constants.ShooterConstants.Left.kdumperLeftMotorkS)
        .kV(Constants.ShooterConstants.Left.kdumperLeftMotorkV)
        .kA(Constants.ShooterConstants.Left.kdumperLeftMotorkA).build();
    Gains dumperRightUpGains = new Gains.Builder()
        .kP(Constants.ShooterConstants.Right.kdumperRightMotorkP)
        .kI(Constants.ShooterConstants.Right.kdumperRightMotorkI)
        .kD(Constants.ShooterConstants.Right.kdumperRightMotorkD)
        .kS(Constants.ShooterConstants.Right.kdumperRightMotorkS)
        .kV(Constants.ShooterConstants.Right.kdumperRightMotorkV)
        .kA(Constants.ShooterConstants.Right.kdumperRightMotorkA).build();
    Gains dumperRightDownGains = new Gains.Builder()
        .kP(Constants.ShooterConstants.Right.kdumperRightMotorkP)
        .kI(Constants.ShooterConstants.Right.kdumperRightMotorkI)
        .kD(Constants.ShooterConstants.Right.kdumperRightMotorkD)
        .kS(Constants.ShooterConstants.Right.kdumperRightMotorkS)
        .kV(Constants.ShooterConstants.Right.kdumperRightMotorkV)
        .kA(Constants.ShooterConstants.Right.kdumperRightMotorkA).build();
    Gains adjustableHoodGains = new Gains.Builder()
        .kP(Constants.ShooterConstants.adjustableHood.kAdjHoodMotorkP)
        .kI(Constants.ShooterConstants.adjustableHood.kAdjHoodMotorkI)
        .kD(Constants.ShooterConstants.adjustableHood.kAdjHoodMotorkD)
        .kS(Constants.ShooterConstants.adjustableHood.kAdjHoodMotorkS)
        .kV(Constants.ShooterConstants.adjustableHood.kAdjHoodMotorkV)
        .kA(Constants.ShooterConstants.adjustableHood.kAdjHoodMotorkA).build();

    setupDumperLeftUp(dumperLeftUpGains);
    setupDumperLeftDown(dumperLeftDownGains);
    setupDumperRightUp(dumperRightUpGains);
    setupDumperRightUp(dumperRightDownGains);
    // setupLeftHood(HoodLeftGains);
    // setupRightHood(HoodRightGains);
    setupAdjustableHood(adjustableHoodGains);
  }

  public void setupDumperLeftUp(Gains g) {
    dumperLeftUpConfig = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.Left.dumperLeftUpID,
        Constants.ShooterConstants.Left.isInverted,
        Constants.ShooterConstants.Left.isCoast,
        Constants.ShooterConstants.Left.statorCurrentLimit,
        Constants.ShooterConstants.Left.supplyCurrentLimit);
    dumperLeftUp = new TalonFX(dumperLeftUpConfig.getMotorInnerId(), new CANBus("rio"));
    dumperLeftUpConfig.configureMotor(dumperLeftUp, g);
    if (Constants.lowTelemetryMode) {
      velDumperLeftUpRequest = shooterFlywheelInnerLeft.getVelocity();
      statorCurrentOfOfDumperLeftUpAmps = shooterFlywheelInnerLeft.getStatorCurrent();
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

  public void setupDumperLeftDown(Gains g) {
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

  public void setupDumperRightUp(Gains g) {
    // Flywheel Configuration
    flywheelConfigOuterRight = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.SharedFlywheel.FlywheelOuterIDRight,
        Constants.ShooterConstants.SharedFlywheel.isInvertedOuterRight,
        Constants.ShooterConstants.SharedFlywheel.isCoastRight,
        Constants.ShooterConstants.SharedFlywheel.statorCurrentLimit,
        Constants.ShooterConstants.SharedFlywheel.supplyCurrentLimit);
    shooterFlywheelOuterRight = new TalonFX(flywheelConfigOuterRight.getMotorInnerId(), new CANBus("rio"));
    flywheelConfigOuterRight.configureMotor(shooterFlywheelOuterRight, g);
    if(Constants.lowTelemetryMode){
    velocityOfMainFlywheelOuterRightRPS = shooterFlywheelInnerRight.getVelocity();
    statorCurrentOfMainFlywheelOuterRightAmps = shooterFlywheelInnerRight.getStatorCurrent();
    flywheelConfigOuterRight.configureSignals(shooterFlywheelOuterRight, 50.0, velocityOfMainFlywheelOuterRightRPS,
        statorCurrentOfMainFlywheelOuterRightAmps);
    }else{
    velocityOfMainFlywheelOuterRightRPS = shooterFlywheelInnerRight.getVelocity();
    statorCurrentOfMainFlywheelOuterRightAmps = shooterFlywheelInnerRight.getStatorCurrent();
    outputOfMainFlywheelOuterRightVolts = shooterFlywheelInnerRight.getMotorVoltage();
    accelerationOfMainFlywheelOuterRight = shooterFlywheelInnerRight.getAcceleration();
    flywheelConfigOuterRight.configureSignals(shooterFlywheelOuterRight, 50.0, velocityOfMainFlywheelOuterRightRPS,
        statorCurrentOfMainFlywheelOuterRightAmps, outputOfMainFlywheelOuterRightVolts,
        accelerationOfMainFlywheelOuterRight);
    }

  }

    public void setupDumperRightDown(Gains g) {
    // Flywheel Configuration
    flywheelConfigOuterLeft = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.SharedFlywheel.FlywheelOuterIDLeft,
        Constants.ShooterConstants.SharedFlywheel.isInvertedOuterLeft,
        Constants.ShooterConstants.SharedFlywheel.isCoastLeft,
        Constants.ShooterConstants.SharedFlywheel.statorCurrentLimit,
        Constants.ShooterConstants.SharedFlywheel.supplyCurrentLimit);
    shooterFlywheelOuterLeft = new TalonFX(flywheelConfigOuterLeft.getMotorInnerId(), new CANBus("rio"));
    flywheelConfigOuterLeft.configureMotor(shooterFlywheelOuterLeft, g);
    if(Constants.lowTelemetryMode){
    velocityOfMainFlywheelOuterLeftRPS = shooterFlywheelOuterLeft.getVelocity();
    statorCurrentOfMainFlywheelOuterLeftAmps = shooterFlywheelOuterLeft.getStatorCurrent();
    flywheelConfigOuterLeft.configureSignals(shooterFlywheelOuterLeft, 50.0, velocityOfMainFlywheelOuterLeftRPS,
        statorCurrentOfMainFlywheelOuterLeftAmps);
    }else{
    velocityOfMainFlywheelOuterLeftRPS = shooterFlywheelOuterLeft.getVelocity();
    statorCurrentOfMainFlywheelOuterLeftAmps = shooterFlywheelOuterLeft.getStatorCurrent();
    outputOfMainFlywheelOuterLeftVolts = shooterFlywheelOuterLeft.getMotorVoltage();
    accelerationOfMainFlywheelOuterLeft = shooterFlywheelOuterLeft.getAcceleration();
    flywheelConfigOuterLeft.configureSignals(shooterFlywheelOuterLeft, 50.0, velocityOfMainFlywheelOuterLeftRPS,
        statorCurrentOfMainFlywheelOuterLeftAmps, outputOfMainFlywheelOuterLeftVolts,
        accelerationOfMainFlywheelOuterLeft);
    }

  }

  // public void setupLeftHood(Gains g) {
  //   // Flywheel Configuration
  //   HoodMConfigLeft = new ModuleConfigurator(g.toSlot0Configs(),
  //       Constants.ShooterConstants.Left.HoodID,
  //       Constants.ShooterConstants.Left.isInverted,
  //       Constants.ShooterConstants.Left.isCoast,
  //       Constants.ShooterConstants.Left.statorCurrentLimit,
  //       Constants.ShooterConstants.Left.supplyCurrentLimit);
  //   HoodWheelMotorLeft = new TalonFX(HoodMConfigLeft.getMotorInnerId(), new CANBus("rio"));
  //   HoodMConfigLeft.configureMotor(HoodWheelMotorLeft, g);
  //   if(Constants.lowTelemetryMode){
  //   velocityOfHoodWheelMotorLeftRPS = HoodWheelMotorLeft.getVelocity();
  //   statorCurrentOfHoodLeftAmps = HoodWheelMotorLeft.getStatorCurrent();
  //   HoodMConfigLeft.configureSignals(HoodWheelMotorLeft, 50.0, velocityOfHoodWheelMotorLeftRPS,
  //       statorCurrentOfHoodLeftAmps);
  //   }else{
  //   velocityOfHoodWheelMotorLeftRPS = HoodWheelMotorLeft.getVelocity();
  //   statorCurrentOfHoodLeftAmps = HoodWheelMotorLeft.getStatorCurrent();
  //   outputOfHoodLeftVolts = HoodWheelMotorLeft.getMotorVoltage();
  //   accelerationOfHoodLeft = HoodWheelMotorLeft.getAcceleration();
  //   HoodMConfigLeft.configureSignals(HoodWheelMotorLeft, 50.0, velocityOfHoodWheelMotorLeftRPS,
  //       statorCurrentOfHoodLeftAmps, outputOfHoodLeftVolts, accelerationOfHoodLeft);
  //   }

  // }

  // public void setupRightHood(Gains g) {
  //   // Flywheel Configuration
  //   HoodMConfigRight = new ModuleConfigurator(g.toSlot0Configs(),
  //       Constants.ShooterConstants.Right.HoodID,
  //       Constants.ShooterConstants.Right.isInverted,
  //       Constants.ShooterConstants.Right.isCoast,
  //       Constants.ShooterConstants.Right.statorCurrentLimit,
  //       Constants.ShooterConstants.Right.supplyCurrentLimit);
  //   HoodWheelMotorRight = new TalonFX(HoodMConfigRight.getMotorInnerId(), new CANBus("rio"));
  //   HoodMConfigRight.configureMotor(HoodWheelMotorRight, g);
  //   if(Constants.lowTelemetryMode){
  //   velocityOfHoodWheelMotorRightRPS = HoodWheelMotorRight.getVelocity();
  //   statorCurrentOfHoodRightAmps = HoodWheelMotorRight.getStatorCurrent();
  //   flywheelConfigLeft.configureSignals(HoodWheelMotorRight, 50.0, velocityOfHoodWheelMotorRightRPS,
  //       statorCurrentOfHoodRightAmps);
  //   }
  //   else{
  //   velocityOfHoodWheelMotorRightRPS = HoodWheelMotorRight.getVelocity();
  //   statorCurrentOfHoodRightAmps = HoodWheelMotorRight.getStatorCurrent();
  //   outputOfHoodRightVolts = HoodWheelMotorRight.getMotorVoltage();
  //   accelerationOfHoodRight = HoodWheelMotorRight.getAcceleration();
  //   flywheelConfigLeft.configureSignals(HoodWheelMotorRight, 50.0, velocityOfHoodWheelMotorRightRPS,
  //       statorCurrentOfHoodRightAmps, outputOfHoodRightVolts, accelerationOfHoodRight);
  //   }

  // }

   public void setupAdjustableHood(Gains g) {
    // Flywheel Configuration
    adjustableHoodConfigurator = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.adjustableHood.ID,
        Constants.ShooterConstants.adjustableHood.isInverted,
        Constants.ShooterConstants.adjustableHood.isCoast,
        Constants.ShooterConstants.adjustableHood.statorCurrentLimit,
        Constants.ShooterConstants.adjustableHood.supplyCurrentLimit);
    adjustableHood = new TalonFX(adjustableHoodConfigurator.getMotorInnerId(), new CANBus("rio"));
    adjustableHoodConfigurator.configureMotor(adjustableHood, g);
    if(Constants.lowTelemetryMode){
    velocityOfAdjustableHoodPositionRPS = adjustableHood.getVelocity();
    statorCurrentOfAdjustableHoodPositionAmps = adjustableHood.getStatorCurrent();
    adjustableHoodConfigurator.configureSignals(adjustableHood, 50.0, velocityOfAdjustableHoodPositionRPS,
        statorCurrentOfAdjustableHoodPositionAmps);
    }
    else{
    velocityOfAdjustableHoodPositionRPS = adjustableHood.getVelocity();
    statorCurrentOfAdjustableHoodPositionAmps = adjustableHood.getStatorCurrent();
    outputOfAdjustableHoodPositionVolts = adjustableHood.getMotorVoltage();
    accelerationOfAdjustableHoodPosition = adjustableHood.getAcceleration();
    adjustableHoodConfigurator.configureSignals(adjustableHood, 50.0, velocityOfAdjustableHoodPositionRPS,
        statorCurrentOfAdjustableHoodPositionAmps, outputOfAdjustableHoodPositionVolts, accelerationOfAdjustableHoodPosition);
    }

  }


  public void updateInputs(ShooterIOInputs inputs) {
    if(Constants.lowTelemetryMode){
      lowTelemetry(inputs);
    }
    else{
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
        velocityOfAdjustableHoodPositionRPS,
        statorCurrentOfHoodLeftAmps,
        statorCurrentOfHoodRightAmps,
        statorCurrentOfMainFlywheelLeftAmps,
        statorCurrentOfMainFlywheelRightAmps,
        statorCurrentOfMainFlywheelOuterRightAmps,
        statorCurrentOfMainFlywheelOuterLeftAmps,
        statorCurrentOfAdjustableHoodPositionAmps
        );

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
    inputs.velocityOfAdjustableHoodPositionRPS = velocityOfAdjustableHoodPositionRPS.getValue()
        .in(Rotations.per(Seconds));
    

    inputs.statorCurrentOfHoodLeftAmps = statorCurrentOfHoodLeftAmps.getValue().in(Amps);
    inputs.statorCurrentOfHoodRightAmps = statorCurrentOfHoodRightAmps.getValue().in(Amps);
    inputs.statorCurrentOfMainFlywheelLeftAmps = statorCurrentOfMainFlywheelLeftAmps.getValue().in(Amps);
    inputs.statorCurrentOfMainFlywheelRightAmps = statorCurrentOfMainFlywheelRightAmps.getValue().in(Amps);
    inputs.statorCurrentOfMainFlywheelOuterRightAmps = statorCurrentOfMainFlywheelOuterRightAmps.getValue().in(Amps);
    inputs.statorCurrentOfMainFlywheelOuterLeftAmps = statorCurrentOfMainFlywheelOuterLeftAmps.getValue().in(Amps);
    inputs.statorCurrentOfAdjustableHoodPositionAmps = statorCurrentOfAdjustableHoodPositionAmps.getValue().in(Amps);

    inputs.HoodWheelMotorRightConnected = HoodWheelMotorRight.isConnected();
    inputs.HoodWheelMotorLeftConnected = HoodWheelMotorLeft.isConnected();
    inputs.shooterFlywheelInnerLeftConnected = shooterFlywheelInnerLeft.isConnected();
    inputs.shooterFlywheelInnerRightConnected = shooterFlywheelInnerRight.isConnected();
    inputs.shooterFlywheelOuterRightConnected = shooterFlywheelOuterRight.isConnected();
    inputs.shooterFlywheelOuterLeftConnected = shooterFlywheelOuterLeft.isConnected();
    inputs.adjustableHoodConnected = adjustableHood.isConnected();

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
    Logger.recordOutput("/Shooter/Hood/LeftSetPointSpeed",HoodSetpointLeft);
    HoodWheelMotorLeft.setControl(velHoodLeftRequest.withVelocity(HoodSetpointLeft));
  }

  public void setHoodSpeedOfRight(double shooterHoodSpeedInRPS) {
    HoodSetpointRight = shooterHoodSpeedInRPS;
    Logger.recordOutput("/Shooter/Hood/RightSetPointSpeed",HoodSetpointRight);
    HoodWheelMotorRight.setControl(velHoodRightRequest.withVelocity(HoodSetpointRight));
  }


  public void holdPosition() {
  }

  @Override
  public void stop(){
    stopMainWheel();
    stopHoodLeftWheel();
    stopHoodRightWheel();
    setVelocity(0,0,0);
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