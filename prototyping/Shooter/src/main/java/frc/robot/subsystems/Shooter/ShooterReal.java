package frc.robot.subsystems.Shooter;

import org.bobcatrobotics.Util.Tunables.TunableDouble;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Minutes;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Modules.ModuleConfigurator;
import frc.robot.subsystems.Shooter.Modules.ModuleType;

public class ShooterReal implements ShooterIO {
  private TalonFX shooterFlywheelInner;
  private TalonFX backspinWheelMotor;
  private TalonFX shooterIntakeMotor;
  private TalonFXConfiguration shooterIntakeConfig = new TalonFXConfiguration();

  // Defines tunable values , particularly for configurations of motors ( IE PIDs
  // )
  private TunableDouble shooterMainMotorsPIDkP;
  private TunableDouble shooterMainMotorsPIDkI;
  private TunableDouble shooterMainMotorsPIDkD;
  private TunableDouble shooterMainMotorsPIDkV;
  private TunableDouble shooterMainMotorsPIDkS;
  private TunableDouble shooterMainMotorsPIDkA;
  private TunableDouble shooterBackspinMotorsPIDkP;
  private TunableDouble shooterBackspinMotorsPIDkI;
  private TunableDouble shooterBackspinMotorsPIDkD;
  private TunableDouble shooterBackspinMotorsPIDkV;
  private TunableDouble shooterBackspinMotorsPIDkS;
  private TunableDouble shooterBackspinMotorsPIDkA;
  private TunableDouble shooterIntakeMotorsPIDkP;
  private TunableDouble shooterIntakeMotorsPIDkI;
  private TunableDouble shooterIntakeMotorsPIDkD;
  private TunableDouble shooterIntakeMotorsPIDkV;
  private TunableDouble shooterIntakeMotorsPIDkS;
  private TunableDouble shooterIntakeMotorsPIDkA;
  private VelocityTorqueCurrentFOC velIntakeRequest = new VelocityTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC velShooterRequest = new VelocityTorqueCurrentFOC(0);
  private VelocityTorqueCurrentFOC velBackspinRequest = new VelocityTorqueCurrentFOC(0);
  // private final DutyCycleOut velIntakeRequest = new DutyCycleOut(0);
  private StatusSignal<AngularVelocity> velocityOfMainFlywhelInnerRPS;
  private StatusSignal<AngularVelocity> velocityOfbackspinWheelMotorRPS;
  private StatusSignal<Current> statorCurrentOfMainFlywheelInnerAmps;
  private StatusSignal<Current> statorCurrentOfBackspinAmps;
  private StatusSignal<Voltage> outputOfMainFlywheelInnerVolts;
  private StatusSignal<Voltage> outputOfBackspinVolts;
  private StatusSignal<AngularAcceleration> accelerationOfMainFlywheelInner;
  private StatusSignal<AngularAcceleration> accelerationOfBackspin;
  private StatusSignal<AngularVelocity> velocityOfIntakeRPS;
  private StatusSignal<Current> statorCurrentOfIntakeAmps;
  private StatusSignal<Voltage> outputOfIntakeVolts;
  private StatusSignal<AngularAcceleration> accelerationOfIntake;

  public ModuleConfigurator flywheelConfig;
  public ModuleConfigurator backspinConfig;
  public ModuleConfigurator intakeConfig;

  public double mainFlywheelSetpoint = 0;
  public double intakeSetpoint = 0;
  public double backspinSetpoint = 0;

  private String name;
  private List<ModuleType> moduleTypes;

  public ShooterReal(String name, List<ModuleType> moduleTypes) {
    this.moduleTypes = moduleTypes;
    this.name = name;

    if (name == "Left") {
      if (moduleTypes.contains(ModuleType.INTAKE)) {
        // left
        shooterIntakeMotorsPIDkP = new TunableDouble("/Shooter/" + name + "/Intake/PID/kP",
            Constants.ShooterConstants.kIntakeMotorkP);
        shooterIntakeMotorsPIDkI = new TunableDouble("/Shooter/" + name + "/Intake/PID/kI",
            Constants.ShooterConstants.kIntakeMotorkI);
        shooterIntakeMotorsPIDkD = new TunableDouble("/Shooter/" + name + "/Intake/PID/kD",
            Constants.ShooterConstants.kIntakeMotorkD);
        shooterIntakeMotorsPIDkV = new TunableDouble("/Shooter/" + name + "/Intake/PID/kV",
            Constants.ShooterConstants.kIntakeMotorkV);
        shooterIntakeMotorsPIDkS = new TunableDouble("/Shooter/" + name + "/Intake/PID/kS",
            Constants.ShooterConstants.kIntakeMotorkS);
        shooterIntakeMotorsPIDkA = new TunableDouble("/Shooter/" + name + "/Intake/PID/kA",
            Constants.ShooterConstants.kIntakeMotorkA);
      }
      if (moduleTypes.contains(ModuleType.FLYWHEEL)) {
        // left
        shooterMainMotorsPIDkP = new TunableDouble("/Shooter/" + name + "/Flywheel/PID/kP",
            Constants.ShooterConstants.Left.kshooterMainkP);
        shooterMainMotorsPIDkI = new TunableDouble("/Shooter/" + name + "/Flywheel/PID/kI",
            Constants.ShooterConstants.Left.kshooterMainkI);
        shooterMainMotorsPIDkD = new TunableDouble("/Shooter/" + name + "/Flywheel/PID/kD",
            Constants.ShooterConstants.Left.kshooterMainkD);
        shooterMainMotorsPIDkV = new TunableDouble("/Shooter/" + name + "/Flywheel/PID/kV",
            Constants.ShooterConstants.Left.kshooterMainkV);
        shooterMainMotorsPIDkS = new TunableDouble("/Shooter/" + name + "/Flywheel/PID/kS",
            Constants.ShooterConstants.Left.kshooterMainkS);
        shooterMainMotorsPIDkA = new TunableDouble("/Shooter/" + name + "/Flywheel/PID/kA",
            Constants.ShooterConstants.Left.kshooterMainkA);
      }
      if (moduleTypes.contains(ModuleType.BACKSPIN)) {
        shooterBackspinMotorsPIDkP = new TunableDouble("/Shooter/" + name + "/Backspin/PID/kP",
            Constants.ShooterConstants.Left.kBackspinMotorkP);
        shooterBackspinMotorsPIDkI = new TunableDouble("/Shooter/" + name + "/Backspin/PID/kI",
            Constants.ShooterConstants.Left.kBackspinMotorkI);
        shooterBackspinMotorsPIDkD = new TunableDouble("/Shooter/" + name + "/Backspin/PID/kD",
            Constants.ShooterConstants.Left.kBackspinMotorkD);
        shooterBackspinMotorsPIDkV = new TunableDouble("/Shooter/" + name + "/Backspin/PID/kV",
            Constants.ShooterConstants.Left.kBackspinMotorkV);
        shooterBackspinMotorsPIDkS = new TunableDouble("/Shooter/" + name + "/Backspin/PID/kS",
            Constants.ShooterConstants.Left.kBackspinMotorkS);
        shooterBackspinMotorsPIDkA = new TunableDouble("/Shooter/" + name + "/Backspin/PID/kA",
            Constants.ShooterConstants.Left.kBackspinMotorkA);
      }

    } else {
      if (moduleTypes.contains(ModuleType.INTAKE)) {
        shooterIntakeMotorsPIDkP = new TunableDouble("/Shooter/" + name + "/Intake/PID/kP",
            Constants.ShooterConstants.kIntakeMotorkP);
        shooterIntakeMotorsPIDkI = new TunableDouble("/Shooter/" + name + "/Intake/PID/kI",
            Constants.ShooterConstants.kIntakeMotorkI);
        shooterIntakeMotorsPIDkD = new TunableDouble("/Shooter/" + name + "/Intake/PID/kD",
            Constants.ShooterConstants.kIntakeMotorkD);
        shooterIntakeMotorsPIDkV = new TunableDouble("/Shooter/" + name + "/Intake/PID/kV",
            Constants.ShooterConstants.kIntakeMotorkV);
        shooterIntakeMotorsPIDkS = new TunableDouble("/Shooter/" + name + "/Intake/PID/kS",
            Constants.ShooterConstants.kIntakeMotorkS);
        shooterIntakeMotorsPIDkA = new TunableDouble("/Shooter/" + name + "/Intake/PID/kA",
            Constants.ShooterConstants.kIntakeMotorkA);
      }

      if (moduleTypes.contains(ModuleType.FLYWHEEL)) {
        // left
        shooterMainMotorsPIDkP = new TunableDouble("/Shooter/" + name + "/Flywheel/PID/kP",
            Constants.ShooterConstants.Right.kshooterMainkP);
        shooterMainMotorsPIDkI = new TunableDouble("/Shooter/" + name + "/Flywheel/PID/kI",
            Constants.ShooterConstants.Right.kshooterMainkI);
        shooterMainMotorsPIDkD = new TunableDouble("/Shooter/" + name + "/Flywheel/PID/kD",
            Constants.ShooterConstants.Right.kshooterMainkD);
        shooterMainMotorsPIDkV = new TunableDouble("/Shooter/" + name + "/Flywheel/PID/kV",
            Constants.ShooterConstants.Right.kshooterMainkV);
        shooterMainMotorsPIDkS = new TunableDouble("/Shooter/" + name + "/Flywheel/PID/kS",
            Constants.ShooterConstants.Right.kshooterMainkS);
        shooterMainMotorsPIDkA = new TunableDouble("/Shooter/" + name + "/Flywheel/PID/kA",
            Constants.ShooterConstants.Right.kshooterMainkA);

      }
      if (moduleTypes.contains(ModuleType.BACKSPIN)) {
        shooterBackspinMotorsPIDkP = new TunableDouble("/Shooter/" + name + "/Backspin/PID/kP",
            Constants.ShooterConstants.Right.kBackspinMotorkP);
        shooterBackspinMotorsPIDkI = new TunableDouble("/Shooter/" + name + "/Backspin/PID/kI",
            Constants.ShooterConstants.Right.kBackspinMotorkI);
        shooterBackspinMotorsPIDkD = new TunableDouble("/Shooter/" + name + "/Backspin/PID/kD",
            Constants.ShooterConstants.Right.kBackspinMotorkD);
        shooterBackspinMotorsPIDkV = new TunableDouble("/Shooter/" + name + "/Backspin/PID/kV",
            Constants.ShooterConstants.Right.kBackspinMotorkV);
        shooterBackspinMotorsPIDkS = new TunableDouble("/Shooter/" + name + "/Backspin/PID/kS",
            Constants.ShooterConstants.Right.kBackspinMotorkS);
        shooterBackspinMotorsPIDkA = new TunableDouble("/Shooter/" + name + "/Backspin/PID/kA",
            Constants.ShooterConstants.Right.kBackspinMotorkA);
      }

    }

    if (moduleTypes.contains(ModuleType.FLYWHEEL)) {
      Slot0Configs flyweelConfigLeft = new Slot0Configs();
      flyweelConfigLeft.kP = shooterMainMotorsPIDkP.get();
      flyweelConfigLeft.kI = shooterMainMotorsPIDkI.get();
      flyweelConfigLeft.kD = shooterMainMotorsPIDkD.get();
      flyweelConfigLeft.kV = shooterMainMotorsPIDkV.get();
      flyweelConfigLeft.kS = shooterMainMotorsPIDkS.get();
      flyweelConfigLeft.kA = shooterMainMotorsPIDkA.get();

      flywheelConfig = new ModuleConfigurator(flyweelConfigLeft, Constants.ShooterConstants.Left.FlywheelInnerIDLeft,
          Constants.ShooterConstants.Left.FlywheelOuterIDLeft, false, false, true, 40);
      shooterFlywheelInner = new TalonFX(flywheelConfig.getMotorInnerId(), new CANBus("rio"));
      configureShooterFlywheel();

      // Apply to signals
      velocityOfMainFlywhelInnerRPS = shooterFlywheelInner.getVelocity();
      statorCurrentOfMainFlywheelInnerAmps = shooterFlywheelInner.getStatorCurrent();
      outputOfMainFlywheelInnerVolts = shooterFlywheelInner.getMotorVoltage();
      accelerationOfMainFlywheelInner = shooterFlywheelInner.getAcceleration();

      // Set polling frequency and optimizations
      BaseStatusSignal.setUpdateFrequencyForAll(50, velocityOfMainFlywhelInnerRPS);

      BaseStatusSignal.setUpdateFrequencyForAll(50, statorCurrentOfMainFlywheelInnerAmps);

      BaseStatusSignal.setUpdateFrequencyForAll(50, outputOfMainFlywheelInnerVolts);

      BaseStatusSignal.setUpdateFrequencyForAll(50, accelerationOfMainFlywheelInner);

      shooterFlywheelInner.optimizeBusUtilization();
    }

    if (moduleTypes.contains(ModuleType.BACKSPIN)) {
      Slot0Configs backspinConfigLeft = new Slot0Configs();
      backspinConfigLeft.kP = shooterBackspinMotorsPIDkP.get();
      backspinConfigLeft.kI = shooterBackspinMotorsPIDkI.get();
      backspinConfigLeft.kD = shooterBackspinMotorsPIDkD.get();
      backspinConfigLeft.kV = shooterBackspinMotorsPIDkV.get();
      backspinConfigLeft.kS = shooterBackspinMotorsPIDkS.get();
      backspinConfigLeft.kA = shooterBackspinMotorsPIDkA.get();
      backspinConfig = new ModuleConfigurator(backspinConfigLeft, Constants.ShooterConstants.Left.BackspinIDLeft, false,
          true, 40);
      backspinWheelMotor = new TalonFX(backspinConfig.getMotorId(), new CANBus("rio"));
      configurebackspinWheelMotor();
      velocityOfbackspinWheelMotorRPS = backspinWheelMotor.getVelocity();
      statorCurrentOfBackspinAmps = backspinWheelMotor.getStatorCurrent();
      outputOfBackspinVolts = backspinWheelMotor.getMotorVoltage();
      accelerationOfBackspin = backspinWheelMotor.getAcceleration();

      BaseStatusSignal.setUpdateFrequencyForAll(50, velocityOfbackspinWheelMotorRPS);
      BaseStatusSignal.setUpdateFrequencyForAll(50, statorCurrentOfBackspinAmps);
      BaseStatusSignal.setUpdateFrequencyForAll(50, outputOfBackspinVolts);
      BaseStatusSignal.setUpdateFrequencyForAll(50, accelerationOfBackspin);
      backspinWheelMotor.optimizeBusUtilization();
    }

    if (moduleTypes.contains(ModuleType.INTAKE)) {
      Slot0Configs intakeConfigLeft = new Slot0Configs();
      intakeConfigLeft.kP = shooterIntakeMotorsPIDkP.get();
      intakeConfigLeft.kI = shooterIntakeMotorsPIDkI.get();
      intakeConfigLeft.kD = shooterIntakeMotorsPIDkD.get();
      intakeConfigLeft.kV = shooterIntakeMotorsPIDkV.get();
      intakeConfigLeft.kS = shooterIntakeMotorsPIDkS.get();
      intakeConfigLeft.kA = shooterIntakeMotorsPIDkA.get();
      intakeConfig = new ModuleConfigurator(intakeConfigLeft, Constants.ShooterConstants.Left.BackspinIDLeft, false,
          true, 40);
      shooterIntakeMotor = new TalonFX(intakeConfig.getMotorInnerId(), new CANBus("rio"));
      configureShooterIntake();

      // Apply to signals
      velocityOfIntakeRPS = shooterIntakeMotor.getVelocity();
      accelerationOfIntake = shooterIntakeMotor.getAcceleration();
      statorCurrentOfIntakeAmps = shooterIntakeMotor.getStatorCurrent();
      outputOfIntakeVolts = shooterIntakeMotor.getMotorVoltage();
      // Set polling frequency and optimizations
      BaseStatusSignal.setUpdateFrequencyForAll(50, velocityOfIntakeRPS);
      BaseStatusSignal.setUpdateFrequencyForAll(50, statorCurrentOfIntakeAmps);
      BaseStatusSignal.setUpdateFrequencyForAll(50, outputOfIntakeVolts);
      BaseStatusSignal.setUpdateFrequencyForAll(50, accelerationOfIntake);
      shooterIntakeMotor.optimizeBusUtilization();
    }

  }

  public String getName() {
    return name;
  }
  public List<ModuleType> getModuleTypes(){
    return moduleTypes;
  }
  public void configurebackspinWheelMotor() {
    Slot0Configs slot0 = new Slot0Configs();
    // left
    slot0.kP = shooterBackspinMotorsPIDkP.get();
    slot0.kI = shooterBackspinMotorsPIDkP.get();
    slot0.kD = shooterBackspinMotorsPIDkP.get();
    slot0.kV = shooterBackspinMotorsPIDkP.get();
    slot0.kS = shooterBackspinMotorsPIDkP.get();
    slot0.kA = shooterBackspinMotorsPIDkP.get();

    // Top motor configurations
    TalonFXConfiguration shooterBackspinWheelConfig = new TalonFXConfiguration();
    backspinWheelMotor.getConfigurator().apply(shooterBackspinWheelConfig); // reset to default
    if (backspinConfig.isInnerInverted()) {
      shooterBackspinWheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      shooterBackspinWheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    if (backspinConfig.isCoast()) {
      shooterBackspinWheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    } else {
      shooterBackspinWheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
    shooterBackspinWheelConfig.Slot0 = slot0;
    shooterBackspinWheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterBackspinWheelConfig.CurrentLimits.StatorCurrentLimit = backspinConfig.getCurrentLimit();
    backspinWheelMotor.getConfigurator().apply(shooterBackspinWheelConfig);
  }

  /**
   * Configures the left and right motors of the "main" flywheel these are the
   * forward bottom most motors.
   */
  public void configureShooterFlywheel() {
    Slot0Configs slot0 = new Slot0Configs();
    // left
    slot0.kP = shooterMainMotorsPIDkP.get();
    slot0.kI = shooterMainMotorsPIDkI.get();
    slot0.kD = shooterMainMotorsPIDkD.get();
    slot0.kV = shooterMainMotorsPIDkV.get();
    slot0.kS = shooterMainMotorsPIDkS.get();

    // Top motor configurations
    TalonFXConfiguration shooterInnerConfig = new TalonFXConfiguration();
    shooterFlywheelInner.getConfigurator().apply(shooterInnerConfig); // reset to default
    if (flywheelConfig.isInnerInverted()) {
      shooterInnerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      shooterInnerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    if (flywheelConfig.isCoast()) {
      shooterInnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    } else {
      shooterInnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
    shooterInnerConfig.Slot0 = slot0;
    shooterInnerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterInnerConfig.CurrentLimits.StatorCurrentLimit = flywheelConfig.getCurrentLimit();
    shooterFlywheelInner.getConfigurator().apply(shooterInnerConfig);
  }

  /**
   * Configures the left and right motors of the "main" flywheel these are the
   * forward bottom most motors.
   */
  public void configureShooterIntake() {
    Slot0Configs slot0 = new Slot0Configs();
    // left
    slot0.kP = shooterIntakeMotorsPIDkP.get();
    slot0.kI = shooterIntakeMotorsPIDkI.get();
    slot0.kD = shooterIntakeMotorsPIDkD.get();
    slot0.kV = shooterIntakeMotorsPIDkV.get();
    slot0.kS = shooterIntakeMotorsPIDkS.get();

    // Top motor configurations
    shooterIntakeMotor.getConfigurator().apply(shooterIntakeConfig); // reset to default
    if (intakeConfig.isInnerInverted()) {
      shooterIntakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      shooterIntakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    if (intakeConfig.isCoast()) {
      shooterIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    } else {
      shooterIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
    shooterIntakeConfig.Slot0 = slot0;
    shooterIntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterIntakeConfig.CurrentLimits.StatorCurrentLimit = intakeConfig.getCurrentLimit();
    shooterIntakeMotor.getConfigurator().apply(shooterIntakeConfig);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    if (moduleTypes.contains(ModuleType.FLYWHEEL)) {
      BaseStatusSignal.refreshAll(
          velocityOfMainFlywhelInnerRPS,
          statorCurrentOfMainFlywheelInnerAmps,
          outputOfMainFlywheelInnerVolts,
          accelerationOfMainFlywheelInner);
      inputs.velocityOfMainFlywheelInnerRPM = velocityOfMainFlywhelInnerRPS.getValue().in(Rotations.per(Minute));
      inputs.accelerationOfMainFlywhelInner = accelerationOfMainFlywheelInner.getValue()
          .in(RotationsPerSecondPerSecond);
      inputs.mainFlywheelInnerStatorCurrent = statorCurrentOfMainFlywheelInnerAmps.getValue().in(Amps);
      inputs.outputOfMainFlywhelInner = outputOfMainFlywheelInnerVolts.getValue().in(Volts);
      inputs.mainFlywhelInnerConected = shooterFlywheelInner.isConnected();
    }

    if (moduleTypes.contains(ModuleType.INTAKE)) {
      BaseStatusSignal.refreshAll(
          velocityOfIntakeRPS,
          statorCurrentOfIntakeAmps,
          outputOfIntakeVolts,
          accelerationOfIntake);

      inputs.velocityOfIntakeWheelMotorRPM = velocityOfIntakeRPS.getValue().in(Rotations.per(Minute));
      inputs.velocityOfIntakeWheelMotorRPMError = mainFlywheelSetpoint - inputs.velocityOfIntakeWheelMotorRPM;
      inputs.outputOfIntakeWheelInner = outputOfIntakeVolts.getValue().in(Volts);
      inputs.accelerationOfIntakeWheelMotor = accelerationOfIntake.getValue().in(RotationsPerSecondPerSecond);
      inputs.mainIntakeStatorCurrent = statorCurrentOfIntakeAmps.getValue().in(Amps);
      inputs.intakeConnected = shooterIntakeMotor.isConnected();
    }

    if (moduleTypes.contains(ModuleType.BACKSPIN)) {
      BaseStatusSignal.refreshAll(
          velocityOfbackspinWheelMotorRPS,
          statorCurrentOfBackspinAmps,
          outputOfBackspinVolts,
          accelerationOfBackspin);

      inputs.velocityOfbackspinWheelMotorRPM = velocityOfbackspinWheelMotorRPS.getValue().in(Rotations.per(Minute));
      inputs.accelerationOfbackspinWheelMotor = accelerationOfBackspin.getValue().in(RotationsPerSecondPerSecond);
      inputs.mainBackspinStatorCurrent = statorCurrentOfMainFlywheelInnerAmps.getValue().in(Amps);
      inputs.outputOfbackspinWheelMotor = outputOfBackspinVolts.getValue().in(Volts);
      inputs.backspinConnected = backspinWheelMotor.isConnected();
    }

  }

  public void setOutput(double shooterOutput, double backspinOutput) {
    shooterFlywheelInner.set(shooterOutput);
    backspinWheelMotor.set(backspinOutput);
  }

  public void setVelocity(ShooterState desiredState) {
    setVelocity(desiredState.getFlywheelSpeed(),
        desiredState.getBackspinSpeed(), desiredState.getIntakeSpeed());
  }

  public void setVelocity(double shooterFlywheelSpeed, double shooterBackspinSpeed, double ShooterIntakeSpeed) {
    if (moduleTypes.contains(ModuleType.FLYWHEEL)) {
      setMainWheelSpeed(shooterFlywheelSpeed);
    }
    if (moduleTypes.contains(ModuleType.BACKSPIN)) {
      setBackspinSpeed(shooterBackspinSpeed);
    }
    if (moduleTypes.contains(ModuleType.INTAKE)) {
      setIntakeSpeed(shooterBackspinSpeed);
    }
  }

  public void setMainWheelSpeed(double shooterFlywheelSpeedInRPM) {
    mainFlywheelSetpoint = shooterFlywheelSpeedInRPM / 60;
    shooterFlywheelInner.setControl(velShooterRequest.withVelocity(mainFlywheelSetpoint));
  }

  public void setBackspinSpeed(double shooterBackspinSpeedInRPM) {
    backspinSetpoint = shooterBackspinSpeedInRPM / 60;
    backspinWheelMotor.setControl(velBackspinRequest.withVelocity(backspinSetpoint));
  }

  public void setIntakeSpeed(double shooterIntakeSpeedInRPM) {
    mainFlywheelSetpoint = shooterIntakeSpeedInRPM / 60;
    shooterIntakeMotor.setControl(velIntakeRequest.withVelocity(mainFlywheelSetpoint));
  }

  public void holdPosition() {
  }

  public void stopMainWheel() {
    mainFlywheelSetpoint = 0;
    shooterFlywheelInner.stopMotor();
  }

  public void stopBackspinWheel() {
    backspinSetpoint = 0;
    backspinWheelMotor.stopMotor();

  }

  public void stopIntakeMotor() {
    intakeSetpoint = 0;
    shooterIntakeMotor.stopMotor();

  }

  @Override
  public void periodic() {
    if (moduleTypes.contains(ModuleType.FLYWHEEL)) {
      if (shooterMainMotorsPIDkP.hasChanged()
          || shooterMainMotorsPIDkI.hasChanged()
          || shooterMainMotorsPIDkD.hasChanged()
          || shooterMainMotorsPIDkS.hasChanged()
          || shooterMainMotorsPIDkV.hasChanged()
          || shooterMainMotorsPIDkA.hasChanged()) {
        configureShooterFlywheel();
      }
    }
    if (moduleTypes.contains(ModuleType.BACKSPIN)) {
      if (shooterBackspinMotorsPIDkP.hasChanged()
          || shooterBackspinMotorsPIDkI.hasChanged()
          || shooterBackspinMotorsPIDkD.hasChanged()
          || shooterBackspinMotorsPIDkS.hasChanged()
          || shooterBackspinMotorsPIDkV.hasChanged()
          || shooterBackspinMotorsPIDkA.hasChanged()) {
        configurebackspinWheelMotor();
      }
    }
    if (moduleTypes.contains(ModuleType.INTAKE)) {
      if (shooterIntakeMotorsPIDkP.hasChanged()
          || shooterIntakeMotorsPIDkI.hasChanged()
          || shooterIntakeMotorsPIDkD.hasChanged()
          || shooterIntakeMotorsPIDkS.hasChanged()
          || shooterIntakeMotorsPIDkV.hasChanged()
          || shooterIntakeMotorsPIDkA.hasChanged()) {
        configureShooterIntake();
      }
    }
  }

  public void simulationPeriodic() {
  }
}