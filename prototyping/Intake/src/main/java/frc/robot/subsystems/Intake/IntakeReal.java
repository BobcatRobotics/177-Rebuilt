package frc.robot.subsystems.Intake;

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
import frc.robot.Constants.IntakeConstants;

public class IntakeReal implements IntakeIO {

  private final TalonFX positionMotor = new TalonFX(10);
  private final TalonFX velocityMotor = new TalonFX(11);

  private final VelocityTorqueCurrentFOC requestVelocity = new VelocityTorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC requestPosition = new PositionTorqueCurrentFOC(0);
  private StatusSignal<AngularVelocity> velocityRPS;
  private StatusSignal<Angle> poositionDeg;

  public IntakeReal() {
    // Velocity motor configurations
    TalonFXConfiguration velocityMotorConfig = new TalonFXConfiguration();
    velocityMotor.getConfigurator().apply(velocityMotorConfig); // reset to default
    velocityMotorConfig.MotorOutput.Inverted = IntakeConstants.intakeMotorInvert;
    velocityMotorConfig.MotorOutput.NeutralMode = IntakeConstants.intakeMotorBrakeMode;
    velocityMotorConfig.Slot0.kP = IntakeConstants.kTopP;
    velocityMotorConfig.Slot0.kV = IntakeConstants.kTopV;
    velocityMotorConfig.Slot0.kS = IntakeConstants.kTopS;
    velocityMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    velocityMotorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.topCurrentLimit;
    velocityMotor.getConfigurator().apply(velocityMotorConfig);

    // Position motor configurations
    TalonFXConfiguration positionMotorConfig = new TalonFXConfiguration();
    positionMotor.getConfigurator().apply(positionMotorConfig); // reset to default
    positionMotorConfig.MotorOutput.Inverted = IntakeConstants.intakeMotorInvert;
    positionMotorConfig.MotorOutput.NeutralMode = IntakeConstants.intakeMotorBrakeMode;
    positionMotorConfig.Slot0.kP = IntakeConstants.kTopP;
    positionMotorConfig.Slot0.kV = IntakeConstants.kTopV;
    positionMotorConfig.Slot0.kS = IntakeConstants.kTopS;
    positionMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    positionMotorConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.topCurrentLimit;
    positionMotor.getConfigurator().apply(positionMotorConfig);
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

  public void setPosition(double pos){
    requestPosition.withPosition(pos);
  }

  public double getVelocity() {
    return velocityMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void stop() {
    velocityMotor.stopMotor();
  }
}
