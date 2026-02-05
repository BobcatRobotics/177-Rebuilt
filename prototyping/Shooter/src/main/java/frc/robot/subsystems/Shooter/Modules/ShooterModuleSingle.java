package frc.robot.subsystems.Shooter.Modules;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOInputs;
import frc.robot.subsystems.Shooter.ShooterState;;

public class ShooterModuleSingle implements ShooterModuleInterface {

  private final TalonFX shooterMainMotorLeft;
  private final TalonFX shooterIntakeWheelMotor;
  private final TalonFX backspinWheelMotor;

  private final VelocityTorqueCurrentFOC velShooterRequest = new VelocityTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velBackspinRequest = new VelocityTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velIntakeRequest = new VelocityTorqueCurrentFOC(0);
  //private final DutyCycleOut velIntakeRequest = new DutyCycleOut(0);
  private StatusSignal<AngularVelocity> velocityOfMainFlywhelLeftRPS;
  private StatusSignal<AngularVelocity> velocityOfbackspinWheelMotorRPS;
  private StatusSignal<AngularVelocity> velocityOfIntakeWheelRPS;
  private StatusSignal<Current> statorCurrentOfMainFlywheelLeftAmps;
  private StatusSignal<Current> statorCurrentOfBackspinAmps;
  private StatusSignal<Current> statorCurrentofIntakeAmps;
  private StatusSignal<Voltage> outputOfMainFlywheelLeftVolts;
  private StatusSignal<Voltage> outputOfBackspinVolts;
  private StatusSignal<Voltage> outputOfIntakeVolts;
  private StatusSignal<AngularAcceleration> accelerationOfMainFlywheelLeft;
  private StatusSignal<AngularAcceleration> accelerationOfBackspin;
  private StatusSignal<AngularAcceleration> accelerationOfIntake;

  public Configurator flywheelConfig;
  public Configurator backspinConfig;
  public Configurator intakeConfig;

  public int shooterIndex;

  public double mainFlywheelSetpoint = 0;
  public double backspinSetpoint = 0;
  public double intakeSetpoint = 0;

  public ShooterModuleSingle(Configurator flywheelConfig, Configurator backspinConfig, Configurator intakeConfig,
      int shooterIndex) {
    this.flywheelConfig = flywheelConfig;
    this.backspinConfig = backspinConfig;
    this.intakeConfig = intakeConfig;
    shooterMainMotorLeft = new TalonFX(flywheelConfig.getMotorLeftId(), new CANBus("rio"));
    shooterIntakeWheelMotor = new TalonFX(intakeConfig.getMotorRightId(), new CANBus("rio"));
    backspinWheelMotor = new TalonFX(backspinConfig.getMotorRightId(), new CANBus("rio"));
    configureShooterFlywheel();
    configureIntakeWheel();
    configurebackspinWheelMotor();

    // Apply to signals
    velocityOfMainFlywhelLeftRPS = shooterMainMotorLeft.getVelocity();
    velocityOfbackspinWheelMotorRPS = backspinWheelMotor.getVelocity();
    velocityOfIntakeWheelRPS = shooterIntakeWheelMotor.getVelocity();
    statorCurrentOfMainFlywheelLeftAmps = shooterMainMotorLeft.getStatorCurrent();
    statorCurrentOfBackspinAmps = backspinWheelMotor.getStatorCurrent();
    statorCurrentofIntakeAmps = shooterIntakeWheelMotor.getStatorCurrent();
    outputOfMainFlywheelLeftVolts = shooterMainMotorLeft.getMotorVoltage();
    outputOfBackspinVolts = backspinWheelMotor.getMotorVoltage();
    outputOfIntakeVolts = shooterIntakeWheelMotor.getMotorVoltage();
    accelerationOfMainFlywheelLeft = shooterMainMotorLeft.getAcceleration();
    accelerationOfBackspin = backspinWheelMotor.getAcceleration();
    accelerationOfIntake = shooterIntakeWheelMotor.getAcceleration();
    // Set polling frequency and optimizations
    BaseStatusSignal.setUpdateFrequencyForAll(250, velocityOfMainFlywhelLeftRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(250, velocityOfbackspinWheelMotorRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(250, velocityOfIntakeWheelRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(250, statorCurrentOfMainFlywheelLeftAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(250, statorCurrentOfBackspinAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(250, statorCurrentofIntakeAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(250, outputOfMainFlywheelLeftVolts);
    BaseStatusSignal.setUpdateFrequencyForAll(250, outputOfBackspinVolts);
    BaseStatusSignal.setUpdateFrequencyForAll(250, outputOfIntakeVolts);
    BaseStatusSignal.setUpdateFrequencyForAll(250, accelerationOfMainFlywheelLeft);
    BaseStatusSignal.setUpdateFrequencyForAll(250, accelerationOfBackspin);
    BaseStatusSignal.setUpdateFrequencyForAll(250, accelerationOfIntake);
    shooterMainMotorLeft.optimizeBusUtilization();
    shooterIntakeWheelMotor.optimizeBusUtilization();
    backspinWheelMotor.optimizeBusUtilization();
  }

  /**
   * Configures the left and right motors of the "main" flywheel these are the
   * forward bottom most motors.
   */
  public void configureShooterFlywheel() {
    // Top motor configurations
    TalonFXConfiguration shooterLeftConfig = new TalonFXConfiguration();
    shooterMainMotorLeft.getConfigurator().apply(shooterLeftConfig); // reset to default
    if (flywheelConfig.isInverted()) {
      shooterLeftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      shooterLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    if (flywheelConfig.isCoast()) {
      shooterLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    } else {
      shooterLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
    shooterLeftConfig.Slot0.kP = flywheelConfig.getSlotConfig().kP;
    shooterLeftConfig.Slot0.kI = flywheelConfig.getSlotConfig().kI;
    shooterLeftConfig.Slot0.kD = flywheelConfig.getSlotConfig().kD;
    shooterLeftConfig.Slot0.kV = flywheelConfig.getSlotConfig().kV;
    shooterLeftConfig.Slot0.kS = flywheelConfig.getSlotConfig().kS;
    shooterLeftConfig.Slot0.kA = flywheelConfig.getSlotConfig().kA;
    shooterLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterLeftConfig.CurrentLimits.StatorCurrentLimit = flywheelConfig.getCurrentLimit();
    shooterMainMotorLeft.getConfigurator().apply(shooterLeftConfig);
  }

  public void configureIntakeWheel() {
    // Bottom motor configurations
    TalonFXConfiguration shooterIntakeWheelMotorConfig = new TalonFXConfiguration();
    shooterIntakeWheelMotor.getConfigurator().apply(shooterIntakeWheelMotorConfig); // reset to default
    if (intakeConfig.isInverted()) {
      shooterIntakeWheelMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      shooterIntakeWheelMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    if (intakeConfig.isCoast()) {
      shooterIntakeWheelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    } else {
      shooterIntakeWheelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
    shooterIntakeWheelMotorConfig.Slot0.kP = intakeConfig.getSlotConfig().kP;
    shooterIntakeWheelMotorConfig.Slot0.kI = intakeConfig.getSlotConfig().kI;
    shooterIntakeWheelMotorConfig.Slot0.kD = intakeConfig.getSlotConfig().kD;
    shooterIntakeWheelMotorConfig.Slot0.kV = intakeConfig.getSlotConfig().kV;
    shooterIntakeWheelMotorConfig.Slot0.kS = intakeConfig.getSlotConfig().kS;
    shooterIntakeWheelMotorConfig.Slot0.kA = intakeConfig.getSlotConfig().kA;
    shooterIntakeWheelMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterIntakeWheelMotorConfig.CurrentLimits.StatorCurrentLimit = intakeConfig.getCurrentLimit();
    shooterIntakeWheelMotor.getConfigurator().apply(shooterIntakeWheelMotorConfig);
  }

  public void configurebackspinWheelMotor() {
    // Top motor configurations
    TalonFXConfiguration shooterBackspinWheelConfig = new TalonFXConfiguration();
    backspinWheelMotor.getConfigurator().apply(shooterBackspinWheelConfig); // reset to default
    if (backspinConfig.isInverted()) {
      shooterBackspinWheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      shooterBackspinWheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    if (backspinConfig.isCoast()) {
      shooterBackspinWheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    } else {
      shooterBackspinWheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
    shooterBackspinWheelConfig.Slot0.kP = backspinConfig.getSlotConfig().kP;
    shooterBackspinWheelConfig.Slot0.kI = backspinConfig.getSlotConfig().kI;
    shooterBackspinWheelConfig.Slot0.kD = backspinConfig.getSlotConfig().kD;
    shooterBackspinWheelConfig.Slot0.kV = backspinConfig.getSlotConfig().kV;
    shooterBackspinWheelConfig.Slot0.kS = backspinConfig.getSlotConfig().kS;
    shooterBackspinWheelConfig.Slot0.kA = backspinConfig.getSlotConfig().kS;
    shooterBackspinWheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterBackspinWheelConfig.CurrentLimits.StatorCurrentLimit = backspinConfig.getCurrentLimit();
    backspinWheelMotor.getConfigurator().apply(shooterBackspinWheelConfig);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocityOfMainFlywhelLeftRPS,velocityOfbackspinWheelMotorRPS,velocityOfIntakeWheelRPS,
        statorCurrentOfMainFlywheelLeftAmps,statorCurrentOfBackspinAmps,statorCurrentofIntakeAmps,
        outputOfMainFlywheelLeftVolts,outputOfBackspinVolts,outputOfIntakeVolts,
        accelerationOfMainFlywheelLeft,accelerationOfBackspin,accelerationOfIntake);

    inputs.velocityOfMainFlywhelLeftRPS = velocityOfMainFlywhelLeftRPS.getValue().in(Rotation.per(Seconds));
    inputs.velocityOfbackspinWheelMotorRPS = velocityOfbackspinWheelMotorRPS.getValue().in(Rotation.per(Seconds));
    inputs.velocityOfIntakeWheelRPS = velocityOfIntakeWheelRPS.getValue().in(Rotation.per(Seconds));

    inputs.velocityOfMainFlywhelLeftRPSError = mainFlywheelSetpoint - velocityOfMainFlywhelLeftRPS.getValue().in(Rotation.per(Seconds));
    inputs.velocityOfbackspinWheelMotorRPSError = backspinSetpoint -  velocityOfbackspinWheelMotorRPS.getValue().in(Rotation.per(Seconds));
    inputs.velocityOfIntakeWheelRPSError = intakeSetpoint - velocityOfIntakeWheelRPS.getValue().in(Rotation.per(Seconds));

    inputs.outputOfMainFlywhelLeft = shooterMainMotorLeft.getMotorVoltage().getValue().in(Volts);
    inputs.outputOfbackspinWheelMotor = backspinWheelMotor.getMotorVoltage().getValue().in(Volts);
    inputs.outputOfIntakeWheel = shooterIntakeWheelMotor.getMotorVoltage().getValue().in(Volts);

    inputs.accelerationOfMainFlywhelLeft = shooterMainMotorLeft.getAcceleration().getValue().in(RotationsPerSecondPerSecond);
    inputs.accelerationOfbackspinWheelMotor = backspinWheelMotor.getAcceleration().getValue().in(RotationsPerSecondPerSecond);
    inputs.accelerationOfIntakeWheel = shooterIntakeWheelMotor.getAcceleration().getValue().in(RotationsPerSecondPerSecond);

    inputs.mainFlywheelLeftStatorCurrent = shooterMainMotorLeft.getStatorCurrent().getValue().in(Amps);
    inputs.mainBackspinStatorCurrent = backspinWheelMotor.getStatorCurrent().getValue().in(Amps);
    inputs.mainIntakeStatorCurrent = shooterIntakeWheelMotor.getStatorCurrent().getValue().in(Amps);

    inputs.backspinConnected = backspinWheelMotor.isConnected();
    inputs.intakeConnected = shooterIntakeWheelMotor.isConnected();
    inputs.mainFlywhelLeftConnected = shooterMainMotorLeft.isConnected();
  }

  public void setOutput(double shooterOutput, double backspinOutput, double angleOutput) {
    shooterMainMotorLeft.set(shooterOutput);

    backspinWheelMotor.set(backspinOutput);
    shooterIntakeWheelMotor.set(angleOutput);
  }

  public void setVelocity(ShooterState desiredState) {
    setVelocity(desiredState.getFlywheelSpeed(shooterIndex), desiredState.getIntakeSpeed(shooterIndex),
        desiredState.getBackspinSpeed(shooterIndex));
  }

  public void setVelocity(double shooterFlywheelSpeed, double shooterIntakeSpeed, double shooterBackspinSpeed) {
    setMainWheelSpeed(shooterFlywheelSpeed);
    setIntakeSpeed(shooterIntakeSpeed);
    setBackspinSpeed(shooterBackspinSpeed);
  }

  public void setMainWheelSpeed(double shooterFlywheelSpeed) {
    mainFlywheelSetpoint = shooterFlywheelSpeed;
    shooterMainMotorLeft.setControl(velShooterRequest.withVelocity(shooterFlywheelSpeed));
  }

  public void setBackspinSpeed(double shooterBackspinSpeed) {
    backspinSetpoint = shooterBackspinSpeed;
    backspinWheelMotor.setControl(velBackspinRequest.withVelocity(shooterBackspinSpeed));
  }

  public void setIntakeSpeed(double shooterIntakeSpeed) {
    intakeSetpoint = shooterIntakeSpeed;
    shooterIntakeWheelMotor.setControl(velIntakeRequest.withVelocity(shooterIntakeSpeed));
    //shooterIntakeWheelMotor.set(shooterIntakeSpeed);
  }

  public void holdPosition() {
  }

  public void stopMainWheel() {
    mainFlywheelSetpoint = 0;
    shooterMainMotorLeft.stopMotor();
  }

  public void stopBackspinWheel() {
    backspinSetpoint = 0;
    backspinWheelMotor.stopMotor();

  }

  public void stopIntakeWheel() {
    intakeSetpoint = 0;
    shooterIntakeWheelMotor.stopMotor();
  }
}