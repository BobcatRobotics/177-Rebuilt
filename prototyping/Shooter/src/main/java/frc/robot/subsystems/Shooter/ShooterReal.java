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
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Modules.ModuleConfigurator;

public class ShooterReal implements ShooterIO {
  private final TalonFX shooterFlywheelInner;
  private final TalonFX shooterFlywheelOuter;
  private final TalonFX backspinWheelMotor;

  // Defines tunable values , particularly for configurations of motors ( IE PIDs
  // )
  private TunableDouble shooterMainMotorsPIDkP;
  private TunableDouble shooterMainMotorsPIDkI;
  private TunableDouble shooterMainMotorsPIDkD;
  private TunableDouble shooterMainMotorsPIDkV;
  private TunableDouble shooterMainMotorsPIDkS;
  private TunableDouble shooterMainMotorsPIDkA;
  private TunableDouble shooterIntakeMotorsPIDkP;
  private TunableDouble shooterIntakeMotorsPIDkI;
  private TunableDouble shooterIntakeMotorsPIDkD;
  private TunableDouble shooterIntakeMotorsPIDkV;
  private TunableDouble shooterIntakeMotorsPIDkS;
  private TunableDouble shooterIntakeMotorsPIDkA;
  private TunableDouble shooterBackspinMotorsPIDkP;
  private TunableDouble shooterBackspinMotorsPIDkI;
  private TunableDouble shooterBackspinMotorsPIDkD;
  private TunableDouble shooterBackspinMotorsPIDkV;
  private TunableDouble shooterBackspinMotorsPIDkS;
  private TunableDouble shooterBackspinMotorsPIDkA;

  private final VelocityTorqueCurrentFOC velShooterRequest = new VelocityTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velBackspinRequest = new VelocityTorqueCurrentFOC(0);
  // private final DutyCycleOut velIntakeRequest = new DutyCycleOut(0);
  private StatusSignal<AngularVelocity> velocityOfMainFlywhelInnerRPS;
  private StatusSignal<AngularVelocity> velocityOfMainFlywhelOuterRPS;
  private StatusSignal<AngularVelocity> velocityOfbackspinWheelMotorRPS;
  private StatusSignal<Current> statorCurrentOfMainFlywheelInnerAmps;
  private StatusSignal<Current> statorCurrentOfMainFlywheelOuterAmps;
  private StatusSignal<Current> statorCurrentOfBackspinAmps;
  private StatusSignal<Voltage> outputOfMainFlywheelInnerVolts;
  private StatusSignal<Voltage> outputOfMainFlywheelOuterVolts;
  private StatusSignal<Voltage> outputOfBackspinVolts;
  private StatusSignal<AngularAcceleration> accelerationOfMainFlywheelInner;
  private StatusSignal<AngularAcceleration> accelerationOfMainFlywheelOuter;
  private StatusSignal<AngularAcceleration> accelerationOfBackspin;
  private StatusSignal<AngularAcceleration> accelerationOfIntake;

  public ModuleConfigurator flywheelConfig;
  public ModuleConfigurator backspinConfig;

  public double mainFlywheelSetpoint = 0;
  public double backspinSetpoint = 0;

  private String name;

  public ShooterReal(String name) {
    this.name = name;

    if (name == "Left") {

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
    } else {

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

    Slot0Configs flyweelConfigLeft = new Slot0Configs();
    flyweelConfigLeft.kP = shooterMainMotorsPIDkP.get();
    flyweelConfigLeft.kI = shooterMainMotorsPIDkI.get();
    flyweelConfigLeft.kD = shooterMainMotorsPIDkD.get();
    flyweelConfigLeft.kV = shooterMainMotorsPIDkV.get();
    flyweelConfigLeft.kS = shooterMainMotorsPIDkS.get();
    flyweelConfigLeft.kA = shooterMainMotorsPIDkA.get();
    Slot0Configs intakeConfigLeft = new Slot0Configs();
    intakeConfigLeft.kP = shooterIntakeMotorsPIDkP.get();
    intakeConfigLeft.kI = shooterIntakeMotorsPIDkI.get();
    intakeConfigLeft.kD = shooterIntakeMotorsPIDkD.get();
    intakeConfigLeft.kV = shooterIntakeMotorsPIDkV.get();
    intakeConfigLeft.kS = shooterIntakeMotorsPIDkS.get();
    intakeConfigLeft.kA = shooterIntakeMotorsPIDkA.get();
    Slot0Configs backspinConfigLeft = new Slot0Configs();
    backspinConfigLeft.kP = shooterBackspinMotorsPIDkP.get();
    backspinConfigLeft.kI = shooterBackspinMotorsPIDkI.get();
    backspinConfigLeft.kD = shooterBackspinMotorsPIDkD.get();
    backspinConfigLeft.kV = shooterBackspinMotorsPIDkV.get();
    backspinConfigLeft.kS = shooterBackspinMotorsPIDkS.get();
    backspinConfigLeft.kA = shooterBackspinMotorsPIDkA.get();
    flywheelConfig = new ModuleConfigurator(flyweelConfigLeft, Constants.ShooterConstants.Left.FlywheelInnerIDLeft,
        Constants.ShooterConstants.Left.FlywheelOuterIDLeft, false, false, true, 40);
    backspinConfig = new ModuleConfigurator(backspinConfigLeft, Constants.ShooterConstants.Left.BackspinIDLeft, false,
        true, 40);
    shooterFlywheelOuter = new TalonFX(flywheelConfig.getMotorOuterId(), new CANBus("rio"));
    shooterFlywheelInner = new TalonFX(flywheelConfig.getMotorInnerId(), new CANBus("rio"));
    backspinWheelMotor = new TalonFX(backspinConfig.getMotorId(), new CANBus("rio"));
    configureShooterFlywheel();
    configurebackspinWheelMotor();

    // Apply to signals
    velocityOfMainFlywhelInnerRPS = shooterFlywheelInner.getVelocity();
    velocityOfMainFlywhelOuterRPS = shooterFlywheelOuter.getVelocity();
    velocityOfbackspinWheelMotorRPS = backspinWheelMotor.getVelocity();
    // Set polling frequency and optimizations
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocityOfMainFlywhelInnerRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocityOfMainFlywhelOuterRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocityOfbackspinWheelMotorRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(50, statorCurrentOfMainFlywheelInnerAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(50, statorCurrentOfMainFlywheelOuterAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(50, statorCurrentOfBackspinAmps);
    BaseStatusSignal.setUpdateFrequencyForAll(50, outputOfMainFlywheelInnerVolts);
    BaseStatusSignal.setUpdateFrequencyForAll(50, outputOfMainFlywheelOuterVolts);
    BaseStatusSignal.setUpdateFrequencyForAll(50, outputOfBackspinVolts);
    BaseStatusSignal.setUpdateFrequencyForAll(50, accelerationOfMainFlywheelInner);
    BaseStatusSignal.setUpdateFrequencyForAll(50, accelerationOfMainFlywheelOuter);
    BaseStatusSignal.setUpdateFrequencyForAll(50, accelerationOfBackspin);
    shooterFlywheelInner.optimizeBusUtilization();
    shooterFlywheelOuter.optimizeBusUtilization();
    backspinWheelMotor.optimizeBusUtilization();

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

    TalonFXConfiguration shooterOuterConfig = new TalonFXConfiguration();
    shooterFlywheelOuter.getConfigurator().apply(shooterOuterConfig); // reset to default
    if (flywheelConfig.isOuterInverted()) {
      shooterOuterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      shooterOuterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }
    if (flywheelConfig.isCoast()) {
      shooterOuterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    } else {
      shooterOuterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
    shooterOuterConfig.Slot0 = slot0;
    shooterOuterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterOuterConfig.CurrentLimits.StatorCurrentLimit = flywheelConfig.getCurrentLimit();
    shooterFlywheelOuter.getConfigurator().apply(shooterOuterConfig);

  }

  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocityOfMainFlywhelInnerRPS, velocityOfMainFlywhelOuterRPS, velocityOfbackspinWheelMotorRPS,
        statorCurrentOfMainFlywheelInnerAmps, statorCurrentOfMainFlywheelOuterAmps, statorCurrentOfBackspinAmps,
        outputOfMainFlywheelInnerVolts, outputOfMainFlywheelOuterVolts, outputOfBackspinVolts,
        accelerationOfMainFlywheelInner, accelerationOfMainFlywheelOuter, accelerationOfBackspin, accelerationOfIntake);

    inputs.velocityOfMainFlywheelInnerRPS = velocityOfMainFlywhelInnerRPS.getValue().in(Rotation.per(Seconds));
    inputs.velocityOfMainFlywheelOuterRPS = velocityOfMainFlywhelOuterRPS.getValue().in(Rotation.per(Seconds));
    inputs.velocityOfbackspinWheelMotorRPS = velocityOfbackspinWheelMotorRPS.getValue().in(Rotation.per(Seconds));

    inputs.backspinConnected = backspinWheelMotor.isConnected();
    inputs.mainFlywhelOuterConnected = shooterFlywheelOuter.isConnected();
    inputs.mainFlywhelInnerConected = shooterFlywheelInner.isConnected();
  }

  public void setOutput(double shooterOutput, double backspinOutput) {
    shooterFlywheelInner.set(shooterOutput);
    shooterFlywheelOuter.set(shooterOutput);

    backspinWheelMotor.set(backspinOutput);
  }

  public void setVelocity(ShooterState desiredState) {
    setVelocity(desiredState.getFlywheelSpeed(),
        desiredState.getBackspinSpeed());
  }

  public void setVelocity(double shooterFlywheelSpeed, double shooterBackspinSpeed) {
    setMainWheelSpeed(shooterFlywheelSpeed);
    setBackspinSpeed(shooterBackspinSpeed);
  }

  public void setMainWheelSpeed(double shooterFlywheelSpeed) {
    mainFlywheelSetpoint = shooterFlywheelSpeed;
    shooterFlywheelInner.setControl(velShooterRequest.withVelocity(shooterFlywheelSpeed));
  }

  public void setBackspinSpeed(double shooterBackspinSpeed) {
    backspinSetpoint = shooterBackspinSpeed;
    backspinWheelMotor.setControl(velBackspinRequest.withVelocity(shooterBackspinSpeed));
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

  @Override
  public void periodic() {

    // left
    if (shooterMainMotorsPIDkP.hasChanged()
        || shooterMainMotorsPIDkI.hasChanged()
        || shooterMainMotorsPIDkD.hasChanged()
        || shooterMainMotorsPIDkS.hasChanged()
        || shooterMainMotorsPIDkV.hasChanged()
        || shooterMainMotorsPIDkA.hasChanged()) {
      configureShooterFlywheel();
    }
    if (shooterBackspinMotorsPIDkP.hasChanged()
        || shooterBackspinMotorsPIDkI.hasChanged()
        || shooterBackspinMotorsPIDkD.hasChanged()
        || shooterBackspinMotorsPIDkS.hasChanged()
        || shooterBackspinMotorsPIDkV.hasChanged()
        || shooterBackspinMotorsPIDkA.hasChanged()) {
      configurebackspinWheelMotor();
    }
  }

  public void simulationPeriodic() {
  }
}