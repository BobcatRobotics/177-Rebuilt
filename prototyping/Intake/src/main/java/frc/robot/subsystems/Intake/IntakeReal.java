package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.IntakeConstants;;

public class IntakeReal implements IntakeIO {

  private final TalonFX positionMotor = new TalonFX(10);
  private final TalonFX velocityMotor = new TalonFX(11);

  private final VelocityTorqueCurrentFOC requestVelocity = new VelocityTorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC requestPosition = new PositionTorqueCurrentFOC(0);
  private StatusSignal<AngularVelocity> velocityRPS;
  private StatusSignal<Angle> poositionDeg;

  public IntakeReal() {
    // Bottom motor configurations
    TalonFXConfiguration bottomConfigs = new TalonFXConfiguration();
    velocityMotor.getConfigurator().apply(bottomConfigs); // reset to default
    bottomConfigs.MotorOutput.Inverted = IntakeConstants.intakeMotorInvert;
    bottomConfigs.MotorOutput.NeutralMode = IntakeConstants.intakeMotorBrakeMode;
    bottomConfigs.Slot0.kP = IntakeConstants.kTopP;
    bottomConfigs.Slot0.kV = IntakeConstants.kTopV;
    bottomConfigs.Slot0.kS = IntakeConstants.kTopS;
    bottomConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    bottomConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.topCurrentLimit;
    velocityMotor.getConfigurator().apply(bottomConfigs);
    // Apply to signals
    velocityRPS = velocityMotor.getVelocity();
    // Set polling frequency and optimizations
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocityRPS);
    positionMotor.optimizeBusUtilization();
    velocityMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocityRPS, poositionDeg);

    inputs.velocityRPM = velocityRPS.getValue().in(Rotation.per(Minute));
    inputs.positionDeg = poositionDeg.getValue().in(Degrees);
    inputs.velocityConnected = velocityMotor.isConnected();
  }

  public void setVelocity(double velocity) {
    requestVelocity.withVelocity(velocity);
  }

  public double getVelocity() {
    return velocityMotor.getVelocity().getValueAsDouble();
  }

  //Runs the velocity motor
  public void runIntake(){
     velocityMotor.runIntake();
     return velocityMotor.CurrentLimits();
  }

  //idle state to Manual state
  public void setDown(){
    positionMotor.setState(State.IDLE);
  }

  public void PositionUp(){
    positionMotor.setState(State.MANUAL);
  }

  @Override
  public void stop() {
    velocityMotor.stopMotor();
  }
}
