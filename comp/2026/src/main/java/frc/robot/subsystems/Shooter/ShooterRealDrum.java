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
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
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

  private MotionMagicExpoVoltage LeftUpRequest = new MotionMagicExpoVoltage(0);
  private MotionMagicExpoVoltage LeftDownRequest = new MotionMagicExpoVoltage(0);
  private MotionMagicExpoVoltage RightUpRequest = new MotionMagicExpoVoltage(0);
  private MotionMagicExpoVoltage RightDownRequest = new MotionMagicExpoVoltage(0);

  // private VelocityTorqueCurrentFOC velHoodLeftRequest = new VelocityTorqueCurrentFOC(0);
  // private VelocityTorqueCurrentFOC velHoodRightRequest = new VelocityTorqueCurrentFOC(0);
  private PositionTorqueCurrentFOC posAdjustableHoodRequest = new PositionTorqueCurrentFOC(0);
  
  private TorqueCurrentFOC characterizationRequestTorqueCurrentFOC = new TorqueCurrentFOC(0);
  private VoltageOut characterizationRequestVoltage = new VoltageOut(0);

  private StatusSignal<AngularVelocity> velocityOfDumperLeftUpRPS;
  private StatusSignal<Current> statorCurrentOfDumperLeftUpAmps;
  private StatusSignal<Voltage> outputOfDumperLeftUpVolts;
  private StatusSignal<AngularAcceleration> accelerationOfDumperLeftUp;

  private StatusSignal<AngularVelocity> velocityOfDumperLeftDownRPS;
  private StatusSignal<Current> statorCurrentOfDumperLeftDownAmps;
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
  private StatusSignal<Current> statorCurrentOfDumperRightDownAmps;
  private StatusSignal<Voltage> outputOfDumperRightDownVolts;
  private StatusSignal<AngularAcceleration> accelerationOfDumperRightDown;

  private StatusSignal<AngularVelocity> velocityOfAdjustableHoodPositionRPS;
  private StatusSignal<Current> statorCurrentOfAdjustableHoodPositionAmps;
  private StatusSignal<Voltage> outputOfAdjustableHoodPositionVolts;
  private StatusSignal<AngularAcceleration> accelerationOfAdjustableHoodPosition;



  public double dumperLeftSetPoint = 0;
  // public double dumperLeftDownSetPoint = 0;
  public double dumperRightSetPoint = 0;
  // public double dumperRightDownSetPoint = 0;
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
    setupDumperRightDown(dumperRightDownGains);
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
        Constants.ShooterConstants.Left.supplyCurrentLimit,
        Constants.ShooterConstants.Left.isSoftLimitsEnabled,
        Constants.ShooterConstants.Left.isMotionMagicEnabled,
        Constants.ShooterConstants.Left.cruiseVelocity,
        Constants.ShooterConstants.Left.expo_kA,
        Constants.ShooterConstants.Left.expo_kV);
    dumperLeftUp = new TalonFX(dumperLeftUpConfig.getMotorInnerId(), new CANBus("rio"));
    dumperLeftUpConfig.configureMotor(dumperLeftUp, g);
    if (Constants.lowTelemetryMode) {
      velocityOfDumperLeftUpRPS = dumperLeftUp.getVelocity();
      statorCurrentOfDumperLeftUpAmps = dumperLeftUp.getStatorCurrent();
      dumperLeftUpConfig.configureSignals(dumperLeftUp, 50.0, velocityOfDumperLeftUpRPS,
          statorCurrentOfDumperLeftUpAmps);
    } else {
      velocityOfDumperLeftUpRPS = dumperLeftUp.getVelocity();
      statorCurrentOfDumperLeftUpAmps = dumperLeftUp.getStatorCurrent();
      outputOfDumperLeftUpVolts = dumperLeftUp.getMotorVoltage();
      accelerationOfDumperLeftUp = dumperLeftUp.getAcceleration();
      dumperLeftUpConfig.configureSignals(dumperLeftUp, 50.0, velocityOfDumperLeftUpRPS,
          statorCurrentOfDumperLeftUpAmps, outputOfDumperLeftUpVolts, accelerationOfDumperLeftUp);
    }

  }

  public void setupDumperLeftDown(Gains g) {
    dumperLeftDownConfig = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.Left.dumperLeftDownID,
        Constants.ShooterConstants.Left.isInverted,
        Constants.ShooterConstants.Left.isCoast,
        Constants.ShooterConstants.Left.statorCurrentLimit,
        Constants.ShooterConstants.Left.supplyCurrentLimit,
        Constants.ShooterConstants.Left.isSoftLimitsEnabled,
        Constants.ShooterConstants.Left.isMotionMagicEnabled,
        Constants.ShooterConstants.Left.cruiseVelocity,
        Constants.ShooterConstants.Left.expo_kA,
        Constants.ShooterConstants.Left.expo_kV);
    dumperLeftDown = new TalonFX(dumperLeftDownConfig.getMotorInnerId(), new CANBus("rio"));
    dumperLeftDownConfig.configureMotor(dumperLeftDown, g);
    if (Constants.lowTelemetryMode) {
      velocityOfDumperLeftDownRPS = dumperLeftDown.getVelocity();
      statorCurrentOfDumperLeftDownAmps = dumperLeftDown.getStatorCurrent();
      dumperLeftDownConfig.configureSignals(dumperLeftDown, 50.0, velocityOfDumperLeftDownRPS,
          statorCurrentOfDumperLeftDownAmps);
    } else {
      velocityOfDumperLeftDownRPS = dumperLeftDown.getVelocity();
      statorCurrentOfDumperLeftDownAmps = dumperLeftDown.getStatorCurrent();
      outputOfDumperLeftDownVolts = dumperLeftDown.getMotorVoltage();
      accelerationOfDumperLeftDown = dumperLeftDown.getAcceleration();
      dumperLeftDownConfig.configureSignals(dumperLeftDown, 50.0, velocityOfDumperLeftDownRPS,
          statorCurrentOfDumperLeftDownAmps, outputOfDumperLeftDownVolts, accelerationOfDumperLeftDown);
    }
  }

  public void setupDumperRightUp(Gains g) {
    dumperRightUpConfig = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.Right.dumperRightUpID,
        Constants.ShooterConstants.Right.isInverted,
        Constants.ShooterConstants.Right.isCoast,
        Constants.ShooterConstants.Right.statorCurrentLimit,
        Constants.ShooterConstants.Right.supplyCurrentLimit,
        Constants.ShooterConstants.Right.isSoftLimitsEnabled,
        Constants.ShooterConstants.Right.isMotionMagicEnabled,
        Constants.ShooterConstants.Right.cruiseVelocity,
        Constants.ShooterConstants.Right.expo_kA,
        Constants.ShooterConstants.Right.expo_kV);
    dumperRightUp = new TalonFX(dumperRightUpConfig.getMotorInnerId(), new CANBus("rio"));
    dumperRightUpConfig.configureMotor(dumperRightUp, g);
    if (Constants.lowTelemetryMode) {
      velocityOfDumperRightUpRPS = dumperRightUp.getVelocity();
      statorCurrentOfOfDumperRightUpAmps = dumperRightUp.getStatorCurrent();
      dumperRightUpConfig.configureSignals(dumperRightUp, 50.0, velocityOfDumperRightUpRPS,
          statorCurrentOfOfDumperRightUpAmps);
    } else {
      velocityOfDumperRightUpRPS = dumperRightUp.getVelocity();
      statorCurrentOfOfDumperRightUpAmps = dumperRightUp.getStatorCurrent();
      outputOfDumperRightUpVolts = dumperRightUp.getMotorVoltage();
      accelerationOfDumperRightUp = dumperRightUp.getAcceleration();
      dumperRightUpConfig.configureSignals(dumperRightUp, 50.0, velocityOfDumperRightUpRPS,
          statorCurrentOfOfDumperRightUpAmps, outputOfDumperRightUpVolts, accelerationOfDumperRightUp);
    }
  }

    public void setupDumperRightDown(Gains g) {
    dumperRightDownConfig = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.Right.dumperRightDownID,
        Constants.ShooterConstants.Right.isInverted,
        Constants.ShooterConstants.Right.isCoast,
        Constants.ShooterConstants.Right.statorCurrentLimit,
        Constants.ShooterConstants.Right.supplyCurrentLimit,
        Constants.ShooterConstants.Right.isSoftLimitsEnabled,
        Constants.ShooterConstants.Right.isMotionMagicEnabled,
        Constants.ShooterConstants.Right.cruiseVelocity,
        Constants.ShooterConstants.Right.expo_kA,
        Constants.ShooterConstants.Right.expo_kV);
    dumperRightDown = new TalonFX(dumperRightDownConfig.getMotorInnerId(), new CANBus("rio"));
    dumperRightDownConfig.configureMotor(dumperRightDown, g);
    if (Constants.lowTelemetryMode) {
      velocityOfDumperRightDownRPS = dumperRightDown.getVelocity();
      statorCurrentOfDumperRightDownAmps = dumperRightDown.getStatorCurrent();
      dumperRightDownConfig.configureSignals(dumperRightDown, 50.0, velocityOfDumperRightDownRPS,
          statorCurrentOfDumperRightDownAmps);
    } else {
      velocityOfDumperRightDownRPS = dumperRightDown.getVelocity();
      statorCurrentOfDumperRightDownAmps = dumperRightDown.getStatorCurrent();
      outputOfDumperRightDownVolts = dumperRightDown.getMotorVoltage();
      accelerationOfDumperRightDown = dumperRightDown.getAcceleration();
      dumperRightDownConfig.configureSignals(dumperRightDown, 50.0, velocityOfDumperRightDownRPS,
          statorCurrentOfDumperRightDownAmps, outputOfDumperRightDownVolts, accelerationOfDumperRightDown);
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
        Constants.ShooterConstants.adjustableHood.supplyCurrentLimit,
        Constants.ShooterConstants.adjustableHood.isSoftLimitsEnabled,
        Constants.ShooterConstants.adjustableHood.isMotionMagicEnabled,
        Constants.ShooterConstants.adjustableHood.cruiseVelocity,
        Constants.ShooterConstants.adjustableHood.expo_kA,
        Constants.ShooterConstants.adjustableHood.expo_kV);
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
        accelerationOfDumperLeftUp,
        accelerationOfDumperLeftDown,
        accelerationOfDumperRightUp,
        accelerationOfDumperRightDown,
        accelerationOfAdjustableHoodPosition,
        // accelerationOfHoodLeft,
        // accelerationOfHoodRight,
        // outputOfHoodLeftVolts,
        // outputOfHoodRightVolts,
        outputOfDumperLeftUpVolts,
        outputOfDumperLeftDownVolts,
        outputOfDumperRightUpVolts,
        outputOfDumperRightDownVolts,
        outputOfAdjustableHoodPositionVolts);

    inputs.accelerationOfDumperLeftUp = accelerationOfDumperLeftUp.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.accelerationOfDumperLeftDown = accelerationOfDumperLeftDown.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.accelerationOfDumperRightUp = accelerationOfDumperRightUp.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.accelerationOfDumperRightDown = accelerationOfDumperRightDown.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.accelerationOfAdjustableHood = accelerationOfAdjustableHoodPosition.getValue()
        .in(RotationsPerSecondPerSecond);

    inputs.outputOfDumperLeftUpVolts = outputOfDumperLeftUpVolts.getValue().in(Volts);
    inputs.outputOfDumperLeftDownVolts = outputOfDumperLeftDownVolts.getValue().in(Volts);
    inputs.outputOfDumperRightUpVolts = outputOfDumperRightUpVolts.getValue().in(Volts);
    inputs.outputOfDumperRightDownVolts = outputOfDumperRightDownVolts.getValue().in(Volts);
    inputs.outputOfAdjustableHoodVolts = outputOfAdjustableHoodPositionVolts.getValue().in(Volts);

    lowTelemetry(inputs);
  }

  public void lowTelemetry(ShooterIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        velocityOfDumperLeftUpRPS,
        velocityOfDumperLeftDownRPS,
        // velocityOfHoodWheelMotorLeftRPS,
        // velocityOfHoodWheelMotorRightRPS,
        velocityOfDumperRightUpRPS,
        velocityOfDumperRightDownRPS,
        velocityOfAdjustableHoodPositionRPS,
        // statorCurrentOfHoodLeftAmps,
        // statorCurrentOfHoodRightAmps,
        statorCurrentOfDumperLeftUpAmps,
        statorCurrentOfDumperLeftDownAmps,
        statorCurrentOfOfDumperRightUpAmps,
        statorCurrentOfDumperRightDownAmps,
        statorCurrentOfAdjustableHoodPositionAmps
        );

    inputs.velocityOfDumperLeftUpRPS = velocityOfDumperLeftUpRPS.getValue().in(Rotations.per(Seconds));
    inputs.velocityOfDumperLeftDownRPS = velocityOfDumperLeftDownRPS.getValue().in(Rotations.per(Seconds));
    inputs.velocityOfDumperRightUpRPS = velocityOfDumperRightUpRPS.getValue()
        .in(Rotations.per(Seconds));
    inputs.velocityOfDumperRightDownRPS = velocityOfDumperRightDownRPS.getValue()
        .in(Rotations.per(Seconds));
    // inputs.velocityOfHoodWheelMotorLeftRPS = velocityOfHoodWheelMotorLeftRPS.getValue()
    //     .in(Rotations.per(Seconds));
    // inputs.velocityOfHoodWheelMotorRightRPS = velocityOfHoodWheelMotorRightRPS.getValue()
    //     .in(Rotations.per(Seconds));
    inputs.velocityOfAdjustableHoodPositionRPS = velocityOfAdjustableHoodPositionRPS.getValue()
        .in(Rotations.per(Seconds));
    

    // inputs.statorCurrentOfHoodLeftAmps = statorCurrentOfHoodLeftAmps.getValue().in(Amps);
    // inputs.statorCurrentOfHoodRightAmps = statorCurrentOfHoodRightAmps.getValue().in(Amps);
    inputs.statorCurrentOfDumperLeftUp = statorCurrentOfDumperLeftUpAmps.getValue().in(Amps);
    inputs.statorCurrentOfDumperLeftDown = statorCurrentOfDumperLeftDownAmps.getValue().in(Amps);
    inputs.statorCurrentOfDumperRightUp = statorCurrentOfOfDumperRightUpAmps.getValue().in(Amps);
    inputs.statorCurrentOfDumperRightDown = statorCurrentOfDumperRightDownAmps.getValue().in(Amps);
    inputs.statorCurrentOfAdjustableHoodPositionAmps = statorCurrentOfAdjustableHoodPositionAmps.getValue().in(Amps);

    inputs.positionOfAdjustableHood = adjustableHood.getPosition().getValueAsDouble();

    // inputs.HoodWheelMotorRightConnected = HoodWheelMotorRight.isConnected();
    // inputs.HoodWheelMotorLeftConnected = HoodWheelMotorLeft.isConnected();
    inputs.DumperLeftUpConnected = dumperLeftUp.isConnected();
    inputs.DumperLeftDownConnected = dumperLeftDown.isConnected();
    inputs.DumperRightUpConnected = dumperRightUp.isConnected();
    inputs.DumperRightDownConnected = dumperRightDown.isConnected();
    inputs.adjustableHoodConnected = adjustableHood.isConnected();

  }

  public void setOutput(double dumperOutput, double hoodPosition) {
    dumperLeftUp.set(dumperOutput);
    dumperLeftDown.set(dumperOutput);
    dumperRightUp.set(dumperOutput);
    dumperRightDown.set(dumperOutput);
    adjustableHood.setPosition(hoodPosition);
  }

  public void setShot(ShooterState desiredState) {
    setVelocity(desiredState.getLeftDumperSpeed(),
        desiredState.getRightDumperSpeed(),
        desiredState.getAdjustableHoodPosition());
  }

  public void setVelocity(double dumperLeftSpeed, double dumperRightSpeed, double adjustableHoodPosition) {
    setDumperLeftSpeed(dumperLeftSpeed);
    setDumperRightSpeed(dumperRightSpeed);
    setAdjustableHoodPosition(adjustableHoodPosition);
  }

  public void setDumperLeftSpeed(double dumperLeftSpeed) {
    dumperLeftSetPoint = dumperLeftSpeed;
    dumperLeftUp.setControl(velDumperLeftUpRequest.withVelocity(dumperLeftSetPoint));
    dumperLeftDown.setControl(velDumperLeftDownRequest.withVelocity(dumperLeftSetPoint));
  }

  public void setDumperRightSpeed(double dumperRightSpeed) {
    dumperRightSetPoint = dumperRightSpeed;
    dumperRightUp.setControl(velDumperRightUpRequest.withVelocity(dumperRightSetPoint));
    dumperRightDown.setControl(velDumperRightDownRequest.withVelocity(dumperRightSetPoint));
  }

  

  public void setAdjustableHoodPosition(double positionOfAdjustableHood) {
    adjustableHoodSetPoint = positionOfAdjustableHood;
    adjustableHood.setControl(posAdjustableHoodRequest.withPosition(positionOfAdjustableHood));
  }


  public void holdPosition() {
  }

  @Override
  public void stop(){
    stopDumperLeft();
    stopDumperRight();
    stopAdjustableHood();
    setVelocity(0,0,0);
  }

  public void stopDumperLeft() {
    dumperLeftSetPoint = 0;
    dumperLeftUp.stopMotor();
    dumperLeftDown.stopMotor();
  }

   public void stopDumperRight() {
    dumperRightSetPoint = 0;
    dumperRightUp.stopMotor();
    dumperRightDown.stopMotor();
  }

  public void stopAdjustableHood() {
    adjustableHoodSetPoint = 0;
    adjustableHood.stopMotor();
  }

  @Override
  public void periodic() {
  }

  public void simulationPeriodic() {
  }

  /* Characterization */
  public void runCharacterization_Flywheel(double output) {
    dumperLeftUp.setControl(switch (ClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
    dumperLeftDown.setControl(switch (ClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
    dumperRightUp.setControl(switch (ClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
    dumperRightDown.setControl(switch (ClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity_Flywheel() {
    double avg = (dumperLeftUp.getVelocity().getValue().in(RotationsPerSecond)
                + dumperLeftDown.getVelocity().getValue().in(RotationsPerSecond)
                + dumperRightUp.getVelocity().getValue().in(RotationsPerSecond)
                + dumperRightDown.getVelocity().getValue().in(RotationsPerSecond)) / 4;
    return avg;
  }

  /* Characterization */
  // public void runCharacterization_Hood(double output) {
  //   HoodWheelMotorLeft.setControl(switch (CharacterizationClosedLoopOutputType.Voltage) {
  //     case Voltage -> characterizationRequestVoltage.withOutput(output);
  //     case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
  //   });
  //   HoodWheelMotorRight.setControl(switch (CharacterizationClosedLoopOutputType.Voltage) {
  //     case Voltage -> characterizationRequestVoltage.withOutput(output);
  //     case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
  //   });
 // }

  // /** Returns the module velocity in rotations/sec (Phoenix native units). */
  // public double getFFCharacterizationVelocity_Hood() {
  //   double avg = (HoodWheelMotorLeft.getVelocity().getValue().in(RotationsPerSecond) +
  //       HoodWheelMotorRight.getVelocity().getValue().in(RotationsPerSecond)) / 2;
  //   return avg;
  // }

}