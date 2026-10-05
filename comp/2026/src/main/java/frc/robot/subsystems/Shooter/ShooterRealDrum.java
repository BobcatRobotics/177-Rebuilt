package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.bobcatrobotics.Util.Tunables.Gains;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.Carwash.CarwashSim;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Modules.ModuleConfigurator;

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
  private MotionMagicVelocityVoltage  velDumperLeftUpRequest = new MotionMagicVelocityVoltage(0);
  private MotionMagicVelocityVoltage  velDumperLeftDownRequest = new MotionMagicVelocityVoltage(0);
  private MotionMagicVelocityVoltage  velDumperRightUpRequest = new MotionMagicVelocityVoltage(0);
  private MotionMagicVelocityVoltage  velDumperRightDownRequest = new MotionMagicVelocityVoltage(0);

  // private VelocityTorqueCurrentFOC velHoodLeftRequest = new VelocityTorqueCurrentFOC(0);
  // private VelocityTorqueCurrentFOC velHoodRightRequest = new VelocityTorqueCurrentFOC(0);
  private MotionMagicExpoVoltage posAdjustableHoodRequest = new MotionMagicExpoVoltage(0);
  
  

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
  public double dumperRightSetPoint = 0;
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
    setupAdjustableHood(adjustableHoodGains);
  }

  public void setupDumperLeftUp(Gains g) {
    dumperLeftUpConfig = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.Left.dumperLeftUpID,
        Constants.ShooterConstants.Left.isInverted,
        Constants.ShooterConstants.Left.isCoast,
        Constants.ShooterConstants.Left.statorCurrentLimit,
        Constants.ShooterConstants.Left.supplyCurrentLimit,
        Constants.ShooterConstants.Left.isSoftLimitsEnabled,Constants.ShooterConstants.Left.useMotionMagic);
    dumperLeftUp = new TalonFX(dumperLeftUpConfig.getMotorInnerId(), new CANBus("rio"));
    dumperLeftUpConfig.applyMotionMagicConfig(
        Constants.ShooterConstants.motionMagicCruiseVelocity,
        Constants.ShooterConstants.motionMagicAcceleration,
        Constants.ShooterConstants.motionMagicJerk,
        Constants.ShooterConstants.motionMagicExpoKV,
        Constants.ShooterConstants.motionMagicExpoKa);
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
        Constants.ShooterConstants.Left.isSoftLimitsEnabled,Constants.ShooterConstants.Left.useMotionMagic);
    dumperLeftDown = new TalonFX(dumperLeftDownConfig.getMotorInnerId(), new CANBus("rio"));
    dumperLeftDownConfig.applyMotionMagicConfig(
        Constants.ShooterConstants.motionMagicCruiseVelocity,
        Constants.ShooterConstants.motionMagicAcceleration,
        Constants.ShooterConstants.motionMagicJerk,
        Constants.ShooterConstants.motionMagicExpoKV,
        Constants.ShooterConstants.motionMagicExpoKa);
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
        Constants.ShooterConstants.Right.isSoftLimitsEnabled,Constants.ShooterConstants.Right.useMotionMagic);
    dumperRightUp = new TalonFX(dumperRightUpConfig.getMotorInnerId(), new CANBus("rio"));
    dumperRightUpConfig.applyMotionMagicConfig(
        Constants.ShooterConstants.motionMagicCruiseVelocity,
        Constants.ShooterConstants.motionMagicAcceleration,
        Constants.ShooterConstants.motionMagicJerk,
        Constants.ShooterConstants.motionMagicExpoKV,
        Constants.ShooterConstants.motionMagicExpoKa);
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
        Constants.ShooterConstants.Right.isSoftLimitsEnabled,Constants.ShooterConstants.Right.useMotionMagic);
    dumperRightDown = new TalonFX(dumperRightDownConfig.getMotorInnerId(), new CANBus("rio"));
      dumperRightDownConfig.applyMotionMagicConfig(
        Constants.ShooterConstants.motionMagicCruiseVelocity,
        Constants.ShooterConstants.motionMagicAcceleration,
        Constants.ShooterConstants.motionMagicJerk,
        Constants.ShooterConstants.motionMagicExpoKV,
        Constants.ShooterConstants.motionMagicExpoKa);
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

   public void setupAdjustableHood(Gains g) {
    // Flywheel Configuration
    adjustableHoodConfigurator = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ShooterConstants.adjustableHood.ID,
        Constants.ShooterConstants.adjustableHood.isInverted,
        Constants.ShooterConstants.adjustableHood.isCoast,
        Constants.ShooterConstants.adjustableHood.statorCurrentLimit,
        Constants.ShooterConstants.adjustableHood.supplyCurrentLimit,
        Constants.ShooterConstants.adjustableHood.isSoftLimitsEnabled,
        Constants.ShooterConstants.adjustableHood.forwardSoftwareLimit,
        Constants.ShooterConstants.adjustableHood.reverseSoftwareLimit,Constants.ShooterConstants.adjustableHood.useMotionMagic);
    adjustableHood = new TalonFX(adjustableHoodConfigurator.getMotorInnerId(), new CANBus("rio"));
    adjustableHoodConfigurator.applyMotionMagicConfig(
        Constants.ShooterConstants.adjustableHood.motionMagicCruiseVelocity,
        Constants.ShooterConstants.adjustableHood.motionMagicAcceleration,
        Constants.ShooterConstants.adjustableHood.motionMagicJerk,
        Constants.ShooterConstants.adjustableHood.motionMagicExpoKV,
        Constants.ShooterConstants.adjustableHood.motionMagicExpoKa);
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
        velocityOfDumperRightUpRPS,
        velocityOfDumperRightDownRPS,
        velocityOfAdjustableHoodPositionRPS,
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
    inputs.velocityOfAdjustableHoodPositionRPS = velocityOfAdjustableHoodPositionRPS.getValue()
        .in(Rotations.per(Seconds));
    
    inputs.statorCurrentOfDumperLeftUp = statorCurrentOfDumperLeftUpAmps.getValue().in(Amps);
    inputs.statorCurrentOfDumperLeftDown = statorCurrentOfDumperLeftDownAmps.getValue().in(Amps);
    inputs.statorCurrentOfDumperRightUp = statorCurrentOfOfDumperRightUpAmps.getValue().in(Amps);
    inputs.statorCurrentOfDumperRightDown = statorCurrentOfDumperRightDownAmps.getValue().in(Amps);
    inputs.statorCurrentOfAdjustableHoodPositionAmps = statorCurrentOfAdjustableHoodPositionAmps.getValue().in(Amps);

    inputs.positionOfAdjustableHood = adjustableHood.getPosition().getValueAsDouble();
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

  // "Shooter/Commanded/*" logs the last value actually sent to the motors each loop. The goal
  // logged in ShooterState.update() runs before commands and can differ from what is sent.
  public void setDumperLeftSpeed(double dumperLeftSpeed) {
    dumperLeftSetPoint = dumperLeftSpeed;
    dumperLeftUp.setControl(velDumperLeftUpRequest.withVelocity(dumperLeftSetPoint));
    dumperLeftDown.setControl(velDumperLeftDownRequest.withVelocity(dumperLeftSetPoint));
    Logger.recordOutput("Shooter/Commanded/LeftDumperRPS", dumperLeftSetPoint);
  }

  public void setDumperRightSpeed(double dumperRightSpeed) {
    dumperRightSetPoint = dumperRightSpeed;
    dumperRightUp.setControl(velDumperRightUpRequest.withVelocity(dumperRightSetPoint));
    dumperRightDown.setControl(velDumperRightDownRequest.withVelocity(dumperRightSetPoint));
    Logger.recordOutput("Shooter/Commanded/RightDumperRPS", dumperRightSetPoint);
  }



  public void setAdjustableHoodPosition(double positionOfAdjustableHood) {
    adjustableHoodSetPoint = positionOfAdjustableHood;
    adjustableHood.setControl(posAdjustableHoodRequest.withPosition(positionOfAdjustableHood));
    Logger.recordOutput("Shooter/Commanded/HoodPositionRotations", adjustableHoodSetPoint);
  }


  public void holdPosition() {
  }

  @Override
  public double getCommandedHoodPosition() {
    return adjustableHoodSetPoint;
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
    Logger.recordOutput("Shooter/Commanded/LeftDumperRPS", dumperLeftSetPoint);
  }

   public void stopDumperRight() {
    dumperRightSetPoint = 0;
    dumperRightUp.stopMotor();
    dumperRightDown.stopMotor();
    Logger.recordOutput("Shooter/Commanded/RightDumperRPS", dumperRightSetPoint);
  }

  public void stopAdjustableHood() {
    adjustableHoodSetPoint = 0;
    adjustableHood.stopMotor();
    Logger.recordOutput("Shooter/Commanded/HoodPositionRotations", adjustableHoodSetPoint);
  }

  @Override
  public void periodic() {
  }

  // ---------------- Simulation only (WPILib never calls simulationPeriodic on the robot) -----------
  // Rough physics so the drum and hood respond in sim. Values are at the rotor (no
  // SensorToMechanismRatio is configured), matching the RPS/rotation units in the shot table.
  private static final double SIM_DRUM_MOI = 0.004; // kg*m^2 per side, reflected to the rotor
  private static final double SIM_HOOD_MOI = 0.0005; // kg*m^2, reflected to the rotor
  // Fake ball load while the carwash is feeding: each ball removes some drum speed
  public static double simBallsPerSecond = 8.0;
  public static double simVelocityLossPerBallRPS = 4.0;

  private FlywheelSim simLeftDrum;
  private FlywheelSim simRightDrum;
  private DCMotorSim simHood;
  private double simBallTimer = 0.0;

  public void simulationPeriodic() {
    if (simLeftDrum == null) {
      DCMotor drumMotors = DCMotor.getKrakenX60(2);
      simLeftDrum = new FlywheelSim(
          LinearSystemId.createFlywheelSystem(drumMotors, SIM_DRUM_MOI, 1.0), drumMotors);
      simRightDrum = new FlywheelSim(
          LinearSystemId.createFlywheelSystem(drumMotors, SIM_DRUM_MOI, 1.0), drumMotors);
      DCMotor hoodMotor = DCMotor.getKrakenX60(1);
      simHood = new DCMotorSim(
          LinearSystemId.createDCMotorSystem(hoodMotor, SIM_HOOD_MOI, 1.0), hoodMotor);
    }
    double dt = 0.02;
    double battery = RobotController.getBatteryVoltage();

    stepDrumSim(simLeftDrum, dumperLeftUp.getSimState(), dumperLeftDown.getSimState(), battery, dt);
    stepDrumSim(simRightDrum, dumperRightUp.getSimState(), dumperRightDown.getSimState(), battery, dt);

    // Ball load: while the carwash is actually feeding forward, knock speed off both drum sides
    // per ball. Uses the speed actually sent to the carwash (CarwashState's goal can read 80 during
    // spin-up even though the command sends -13).
    if (CarwashSim.simCommandedVelocityRps > 40.0) {
      simBallTimer += dt;
      if (simBallTimer >= 1.0 / simBallsPerSecond) {
        simBallTimer = 0.0;
        double loss = Units.rotationsToRadians(simVelocityLossPerBallRPS);
        // Slow toward zero in either direction (the right side is inverted, so its sim spins negative)
        simLeftDrum.setAngularVelocity(slowBy(simLeftDrum.getAngularVelocityRadPerSec(), loss));
        simRightDrum.setAngularVelocity(slowBy(simRightDrum.getAngularVelocityRadPerSec(), loss));
      }
    } else {
      simBallTimer = 0.0;
    }

    TalonFXSimState hoodState = adjustableHood.getSimState();
    hoodState.setSupplyVoltage(battery);
    simHood.setInputVoltage(hoodState.getMotorVoltage());
    simHood.update(dt);
    hoodState.setRawRotorPosition(simHood.getAngularPositionRotations());
    hoodState.setRotorVelocity(Units.radiansToRotations(simHood.getAngularVelocityRadPerSec()));
  }

  private static double slowBy(double velocity, double loss) {
    return Math.signum(velocity) * Math.max(0.0, Math.abs(velocity) - loss);
  }

  private static void stepDrumSim(
      FlywheelSim sim, TalonFXSimState up, TalonFXSimState down, double battery, double dt) {
    up.setSupplyVoltage(battery);
    down.setSupplyVoltage(battery);
    // Both motors drive the same side, so average their output voltage
    sim.setInputVoltage((up.getMotorVoltage() + down.getMotorVoltage()) / 2.0);
    sim.update(dt);
    double rotorRps = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
    up.setRotorVelocity(rotorRps);
    down.setRotorVelocity(rotorRps);
    up.addRotorPosition(rotorRps * dt);
    down.addRotorPosition(rotorRps * dt);
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


  public void resetEncoder() {
    adjustableHood.setPosition(0);
  }

}