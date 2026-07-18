package frc.robot.subsystems.shooter;

import org.bobcatrobotics.Util.Tunables.Gains;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.subsystems.shooter.Modules.ModuleConfigurator;

public class ShooterReal implements ShooterIO {
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

  // private VelocityTorqueCurrentFOC velHoodLeftRequest = new
  // VelocityTorqueCurrentFOC(0);
  // private VelocityTorqueCurrentFOC velHoodRightRequest = new
  // VelocityTorqueCurrentFOC(0);
  private MotionMagicExpoVoltage posAdjustableHoodRequest = new MotionMagicExpoVoltage(0);
  private MotionMagicVelocityVoltage velDumperLeftUpRequest = new MotionMagicVelocityVoltage(0);
  private MotionMagicVelocityVoltage velDumperLeftDownRequest = new MotionMagicVelocityVoltage(0);
  private MotionMagicVelocityVoltage velDumperRightUpRequest = new MotionMagicVelocityVoltage(0);
  private MotionMagicVelocityVoltage velDumperRightDownRequest = new MotionMagicVelocityVoltage(0);

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

  public ShooterReal() {

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
        .kP(Constants.ShooterConstants.AdjustableHood.kAdjHoodMotorkP)
        .kI(Constants.ShooterConstants.AdjustableHood.kAdjHoodMotorkI)
        .kD(Constants.ShooterConstants.AdjustableHood.kAdjHoodMotorkD)
        .kS(Constants.ShooterConstants.AdjustableHood.kAdjHoodMotorkS)
        .kV(Constants.ShooterConstants.AdjustableHood.kAdjHoodMotorkV)
        .kA(Constants.ShooterConstants.AdjustableHood.kAdjHoodMotorkA).build();

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
        Constants.ShooterConstants.Left.isSoftLimitsEnabled, Constants.ShooterConstants.Left.useMotionMagic);
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
        Constants.ShooterConstants.Left.isSoftLimitsEnabled, Constants.ShooterConstants.Left.useMotionMagic);
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
        Constants.ShooterConstants.Right.isSoftLimitsEnabled, Constants.ShooterConstants.Right.useMotionMagic);
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
        Constants.ShooterConstants.Right.isSoftLimitsEnabled, Constants.ShooterConstants.Right.useMotionMagic);
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
        Constants.ShooterConstants.AdjustableHood.ID,
        Constants.ShooterConstants.AdjustableHood.isInverted,
        Constants.ShooterConstants.AdjustableHood.isCoast,
        Constants.ShooterConstants.AdjustableHood.statorCurrentLimit,
        Constants.ShooterConstants.AdjustableHood.supplyCurrentLimit,
        Constants.ShooterConstants.AdjustableHood.isSoftLimitsEnabled,
        Constants.ShooterConstants.AdjustableHood.forwardSoftwareLimit,
        Constants.ShooterConstants.AdjustableHood.reverseSoftwareLimit,
        Constants.ShooterConstants.AdjustableHood.useMotionMagic);
    adjustableHood = new TalonFX(adjustableHoodConfigurator.getMotorInnerId(), new CANBus("rio"));
    adjustableHoodConfigurator.applyMotionMagicConfig(
        Constants.ShooterConstants.AdjustableHood.motionMagicCruiseVelocity,
        Constants.ShooterConstants.AdjustableHood.motionMagicAcceleration,
        Constants.ShooterConstants.AdjustableHood.motionMagicJerk,
        Constants.ShooterConstants.AdjustableHood.motionMagicExpoKV,
        Constants.ShooterConstants.AdjustableHood.motionMagicExpoKa);
    adjustableHoodConfigurator.configureMotor(adjustableHood, g);
    if (Constants.lowTelemetryMode) {
      velocityOfAdjustableHoodPositionRPS = adjustableHood.getVelocity();
      statorCurrentOfAdjustableHoodPositionAmps = adjustableHood.getStatorCurrent();
      adjustableHoodConfigurator.configureSignals(adjustableHood, 50.0, velocityOfAdjustableHoodPositionRPS,
          statorCurrentOfAdjustableHoodPositionAmps);
    } else {
      velocityOfAdjustableHoodPositionRPS = adjustableHood.getVelocity();
      statorCurrentOfAdjustableHoodPositionAmps = adjustableHood.getStatorCurrent();
      outputOfAdjustableHoodPositionVolts = adjustableHood.getMotorVoltage();
      accelerationOfAdjustableHoodPosition = adjustableHood.getAcceleration();
      adjustableHoodConfigurator.configureSignals(adjustableHood, 50.0, velocityOfAdjustableHoodPositionRPS,
          statorCurrentOfAdjustableHoodPositionAmps, outputOfAdjustableHoodPositionVolts,
          accelerationOfAdjustableHoodPosition);
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
        statorCurrentOfAdjustableHoodPositionAmps);

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

  public boolean atSpeed(double targetSpeed) {
    boolean isAtTolerance = false;
    boolean isDumperLeftWithinTolerance = false;
    boolean isDumperRightWithinTolerance = false;
    double mainSpeedTolerance = 5;
    isDumperLeftWithinTolerance = Math.abs(velocityOfDumperLeftDownRPS.getValueAsDouble()
        - targetSpeed) <= mainSpeedTolerance;
    isDumperRightWithinTolerance = Math.abs(velocityOfDumperRightDownRPS.getValueAsDouble()
        - targetSpeed) <= mainSpeedTolerance;
    if (isDumperLeftWithinTolerance && isDumperRightWithinTolerance) {
      isAtTolerance = true;
    }
    return isAtTolerance;
  }

  public void periodic() {

  }

  public void simulationPeriodic() {
  }

  public void runMotors(ShooterState currentState) {
    setVelocity(currentState);
    setPosition(currentState);
  }

  public void setVelocity(ShooterState currentState) {
    dumperLeftUp.setControl(velDumperLeftUpRequest.withVelocity(currentState.getRollerSpeed()));
    dumperLeftDown.setControl(velDumperLeftDownRequest.withVelocity(currentState.getRollerSpeed()));
    dumperRightUp.setControl(velDumperRightUpRequest.withVelocity(currentState.getRollerSpeed()));
    dumperRightDown.setControl(velDumperRightDownRequest.withVelocity(currentState.getRollerSpeed()));
  }

  public void setPosition(ShooterState currentState) {
    adjustableHood.setControl(posAdjustableHoodRequest.withPosition(currentState.getPosition()));
  }

  public void stop() {
    dumperLeftUp.stopMotor();
    dumperLeftDown.stopMotor();
    dumperRightUp.stopMotor();
    dumperRightDown.stopMotor();
    adjustableHood.stopMotor();
  }


  public boolean atSpeed(ShooterState currentState) {
    boolean isAtTolerance = false;
    boolean isDumperLeftWithinTolerance = false;
    boolean isDumperRightWithinTolerance = false;

    double MAIN_SPEED_TOLERANCE = 5; // Using for both dumpers
    double leftVelocityAvg = (dumperLeftUp.getVelocity().getValueAsDouble() + dumperLeftDown.getVelocity().getValueAsDouble())/2;
    double rightVelocityAvg =(dumperRightUp.getVelocity().getValueAsDouble() + dumperRightDown.getVelocity().getValueAsDouble())/2;
    isDumperLeftWithinTolerance = Math.abs( leftVelocityAvg
        - currentState.getRollerSpeed()) <= MAIN_SPEED_TOLERANCE;
    isDumperRightWithinTolerance = Math
        .abs(rightVelocityAvg
            - currentState.getRollerSpeed()) <= MAIN_SPEED_TOLERANCE;
    if (isDumperLeftWithinTolerance && isDumperRightWithinTolerance) {
      isAtTolerance = true;
    }
    Logger.recordOutput("Shooter/isUpToSpeed", isAtTolerance);
    return isAtTolerance;
  }

  public boolean atPosition(ShooterState currentState) {
    boolean isAtTolerance = false;
    boolean isAdjustableHoodWithinTolerance = false;

    double HOOD_POSITION_TOLERANCE = 1;
    double hoodpPosition = adjustableHood.getPosition().getValueAsDouble();
    isAdjustableHoodWithinTolerance = Math
        .abs(hoodpPosition
            - currentState.getPosition()) <= HOOD_POSITION_TOLERANCE;
    if (isAdjustableHoodWithinTolerance) {
      isAtTolerance = true;
    }
    Logger.recordOutput("Shooter/isAtPosition", isAtTolerance);
    return isAtTolerance;
  }
}
