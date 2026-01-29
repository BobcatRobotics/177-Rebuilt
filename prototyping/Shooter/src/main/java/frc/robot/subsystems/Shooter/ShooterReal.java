package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Seconds;

import org.bobcatrobotics.Util.Tunables.TunableDouble;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;;

public class ShooterReal implements ShooterIO {

  private final TalonFX shooterMainMotorLeft = new TalonFX(17);
  private final TalonFX shooterMainMotorRight = new TalonFX(22);

  private final TalonFX shooterIntakeWheelMotor = new TalonFX(30);
  private final TalonFX backspinWheelMotor = new TalonFX(2);

  private final VelocityTorqueCurrentFOC velShooterRequest = new VelocityTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velBackspinRequest = new VelocityTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velIntakeRequest = new VelocityTorqueCurrentFOC(0);
  private StatusSignal<AngularVelocity> velocityOfMainFlywheeRightRPS;
  private StatusSignal<AngularVelocity> velocityOfMainFlywhelLeftRPS;
  private StatusSignal<AngularVelocity> velocityOfbackspinWheelMotorRPS;
  private StatusSignal<AngularVelocity> velocityOfIntakeWheelRPS;

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

  public ShooterReal() {

    shooterMainMotorsPIDkP = new TunableDouble("Shooter/MainMotor/PID/kP", Constants.ShooterConstants.kshooterMainP);
    shooterMainMotorsPIDkI = new TunableDouble("Shooter/MainMotor/PID/kI", Constants.ShooterConstants.kshooterMainI);
    shooterMainMotorsPIDkD = new TunableDouble("Shooter/MainMotor/PID/kD", Constants.ShooterConstants.kshooterMainD);
    shooterMainMotorsPIDkV = new TunableDouble("Shooter/MainMotor/PID/kV", Constants.ShooterConstants.kshooterMainV);
    shooterMainMotorsPIDkS = new TunableDouble("Shooter/MainMotor/PID/kS", Constants.ShooterConstants.kshooterMainS);
    shooterMainMotorsPIDkA = new TunableDouble("Shooter/MainMotor/PID/kA", Constants.ShooterConstants.kshooterMainA);

    shooterIntakeMotorsPIDkP = new TunableDouble("Shooter/Intake/PID/kP", Constants.ShooterConstants.kTopBottomP);
    shooterIntakeMotorsPIDkV = new TunableDouble("Shooter/Intake/PID/kV", Constants.ShooterConstants.kTopBottomV);
    shooterIntakeMotorsPIDkS = new TunableDouble("Shooter/Intake/PID/kS", Constants.ShooterConstants.kTopBottomS);

    shooterBackspinMotorsPIDkP = new TunableDouble("Shooter/BackSpin/PID/kP", Constants.ShooterConstants.kTopTopP);
    shooterBackspinMotorsPIDkV = new TunableDouble("Shooter/BackSpin/PID/kV", Constants.ShooterConstants.kTopTopV);
    shooterBackspinMotorsPIDkS = new TunableDouble("Shooter/BackSpin/PID/kS", Constants.ShooterConstants.kTopTopS);

    configureShooterFlywheel();
    configureIntakeWheel();
    configurebackspinWheelMotor();

    // Apply to signals
    velocityOfMainFlywheeRightRPS = shooterMainMotorRight.getVelocity();
    velocityOfMainFlywhelLeftRPS = shooterMainMotorLeft.getVelocity();
    velocityOfbackspinWheelMotorRPS = backspinWheelMotor.getVelocity();
    velocityOfIntakeWheelRPS = shooterIntakeWheelMotor.getVelocity();
    // Set polling frequency and optimizations
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocityOfMainFlywheeRightRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocityOfMainFlywhelLeftRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocityOfbackspinWheelMotorRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocityOfIntakeWheelRPS);
    shooterMainMotorLeft.optimizeBusUtilization();
    shooterMainMotorRight.optimizeBusUtilization();
    shooterIntakeWheelMotor.optimizeBusUtilization();
    backspinWheelMotor.optimizeBusUtilization();
    Logger.recordOutput("Motor/TopTopWheel/Licensed", backspinWheelMotor.getIsProLicensed().getValue());
    Logger.recordOutput("Motor/TopBottomWheel/Licensed", shooterIntakeWheelMotor.getIsProLicensed().getValue());
    Logger.recordOutput("Motor/BottomLeft/Licensed", shooterMainMotorLeft.getIsProLicensed().getValue());
    Logger.recordOutput("Motor/BottomRight/Licensed", shooterMainMotorRight.getIsProLicensed().getValue());
  }

  /**
   * Configures the left and right motors of the "main" flywheel these are the
   * forward bottom most motors.
   */
  public void configureShooterFlywheel() {
    // Top motor configurations
    TalonFXConfiguration shooterLeftConfig = new TalonFXConfiguration();
    shooterMainMotorLeft.getConfigurator().apply(shooterLeftConfig); // reset to default
    shooterLeftConfig.MotorOutput.Inverted = ShooterConstants.shooterMainMotorLeftInvert;
    shooterLeftConfig.MotorOutput.NeutralMode = ShooterConstants.shooterMainMotorLeftBrakeMode;
    shooterLeftConfig.Slot0.kP = shooterMainMotorsPIDkP.get();
    shooterLeftConfig.Slot0.kI = shooterMainMotorsPIDkI.get();
    shooterLeftConfig.Slot0.kD = shooterMainMotorsPIDkD.get();
    shooterLeftConfig.Slot0.kV = shooterMainMotorsPIDkV.get();
    shooterLeftConfig.Slot0.kS = shooterMainMotorsPIDkS.get();
    shooterLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterLeftConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.bottomLeftCurrentLimit;
    shooterMainMotorLeft.getConfigurator().apply(shooterLeftConfig);
    // Bottom motor configurations
    TalonFXConfiguration shooterRightConfig = new TalonFXConfiguration();
    shooterMainMotorRight.getConfigurator().apply(shooterRightConfig); // reset to default
    shooterRightConfig.MotorOutput.Inverted = ShooterConstants.shooterMainMotorRightInvert;
    shooterRightConfig.MotorOutput.NeutralMode = ShooterConstants.shooterMainMotorRightBrakeMode;
    shooterRightConfig.Slot0.kP = shooterMainMotorsPIDkP.get();
    shooterRightConfig.Slot0.kI = shooterMainMotorsPIDkI.get();
    shooterRightConfig.Slot0.kD = shooterMainMotorsPIDkD.get();
    shooterRightConfig.Slot0.kV = shooterMainMotorsPIDkV.get();
    shooterRightConfig.Slot0.kS = shooterMainMotorsPIDkS.get();
    shooterRightConfig.Slot0.kA = shooterMainMotorsPIDkA.get();
    shooterRightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterRightConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.bottomRightCurrentLimit;
    shooterMainMotorRight.getConfigurator().apply(shooterRightConfig);
  }

  public void configureIntakeWheel() {
    // Bottom motor configurations
    TalonFXConfiguration shooterIntakeWheelMotorConfig = new TalonFXConfiguration();
    shooterIntakeWheelMotor.getConfigurator().apply(shooterIntakeWheelMotorConfig); // reset to default
    shooterIntakeWheelMotorConfig.MotorOutput.Inverted = ShooterConstants.topBottomMotorInvert;
    shooterIntakeWheelMotorConfig.MotorOutput.NeutralMode = ShooterConstants.topBottomMotorBrakeMode;
    shooterIntakeWheelMotorConfig.Slot0.kP = shooterIntakeMotorsPIDkP.get();
    shooterIntakeWheelMotorConfig.Slot0.kV = shooterIntakeMotorsPIDkV.get();
    shooterIntakeWheelMotorConfig.Slot0.kS = shooterIntakeMotorsPIDkS.get();
    shooterIntakeWheelMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterIntakeWheelMotorConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.topBottomCurrentLimit;
    shooterIntakeWheelMotor.getConfigurator().apply(shooterIntakeWheelMotorConfig);
  }

  public void configurebackspinWheelMotor() {
    // Top motor configurations
    TalonFXConfiguration shooterBackspinWheelConfig = new TalonFXConfiguration();
    backspinWheelMotor.getConfigurator().apply(shooterBackspinWheelConfig); // reset to default
    shooterBackspinWheelConfig.MotorOutput.Inverted = ShooterConstants.topTopMotorInvert;
    shooterBackspinWheelConfig.MotorOutput.NeutralMode = ShooterConstants.topTopMotorBrakeMode;
    shooterBackspinWheelConfig.Slot0.kP = shooterBackspinMotorsPIDkP.get();
    shooterBackspinWheelConfig.Slot0.kV = shooterBackspinMotorsPIDkV.get();
    shooterBackspinWheelConfig.Slot0.kS = shooterBackspinMotorsPIDkS.get();
    shooterBackspinWheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterBackspinWheelConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.topTopCurrentLimit;
    backspinWheelMotor.getConfigurator().apply(shooterBackspinWheelConfig);

  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocityOfMainFlywhelLeftRPS, velocityOfMainFlywheeRightRPS);

    inputs.velocityOfMainFlywhelLeftRPS = velocityOfMainFlywhelLeftRPS.getValue().in(Rotation.per(Seconds));

    inputs.velocityOfMainFlywhelRightRPS = velocityOfMainFlywheeRightRPS.getValue().in(Rotation.per(Seconds));

    inputs.velocityOfbackspinWheelMotorRPS = velocityOfbackspinWheelMotorRPS.getValue().in(Rotation.per(Seconds));

    inputs.velocityOfIntakeWheelRPS = velocityOfIntakeWheelRPS.getValue().in(Rotation.per(Seconds));

    inputs.backspinConnected = backspinWheelMotor.isConnected();
    inputs.intakeConnected = shooterIntakeWheelMotor.isConnected();
    inputs.mainFlywhelLeftConnected = shooterMainMotorRight.isConnected();
    inputs.mainFlywhelRightConnected = shooterMainMotorRight.isConnected();
  }

  public void setOutput(double shooterOutput, double backspinOutput, double angleOutput) {
    shooterMainMotorLeft.set(shooterOutput);
    shooterMainMotorRight.set(shooterOutput);

    backspinWheelMotor.set(backspinOutput);
    shooterIntakeWheelMotor.set(angleOutput);
  }

  public void setVelocity(ShooterState desiredState) {
    setVelocity(desiredState.getFlywheelSpeed(), desiredState.getIntakeSpeed(), desiredState.getBackspinSpeed());
  }

  public void setVelocity(double shooterFlywheelSpeed, double shooterIntakeSpeed, double shooterBackspinSpeed) {
    setMainWheelSpeed(shooterFlywheelSpeed);
    setIntakeSpeed(shooterIntakeSpeed);
    setBackspinSpeed(shooterBackspinSpeed);
  }

  public void setMainWheelSpeed(double shooterFlywheelSpeed) {
    shooterMainMotorLeft.setControl(velShooterRequest.withVelocity(shooterFlywheelSpeed));
    shooterMainMotorRight.setControl(velShooterRequest.withVelocity(shooterFlywheelSpeed));
  }

  public void setBackspinSpeed(double shooterBackspinSpeed) {
    backspinWheelMotor.setControl(velBackspinRequest.withVelocity(shooterBackspinSpeed));
  }

  public void setIntakeSpeed(double shooterIntakeSpeed) {
    shooterIntakeWheelMotor.setControl(velIntakeRequest.withVelocity(shooterIntakeSpeed));
  }

  public void holdPosition() {
  }

  @Override
  public void stop() {
    stopMainWheel();

    stopBackspinWheel();

    stopIntakeWheel();
  }

  public void stopMainWheel() {
    shooterMainMotorLeft.stopMotor();
    shooterMainMotorRight.stopMotor();
  }

  public void stopBackspinWheel() {
    backspinWheelMotor.stopMotor();

  }

  public void stopIntakeWheel() {
    shooterIntakeWheelMotor.stopMotor();
  }

  public void periodic() {
  }

}