package frc.robot.subsystems.Turret;

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
import frc.robot.Constants.TurretConstants;;

public class TurretReal implements TurretIO {

  private final TalonFX positionMotor = new TalonFX(10);
  private final TalonFX velocityMotor = new TalonFX(id); // Add id
  private final PositionTorqueCurrentFOC requestPosition = new PositionTorqueCurrentFOC(0);
  private StatusSignal<AngularVelocity> velocityRPS;
  private StatusSignal<Angle> poositionDeg;

  public TurretReal() {
    // Top motor configurations
    TalonFXConfiguration topConfigs = new TalonFXConfiguration();
    positionMotor.getConfigurator().apply(topConfigs); // reset to default
    topConfigs.MotorOutput.Inverted = TurretConstants.intakeMotorInvert;
    topConfigs.MotorOutput.NeutralMode = TurretConstants.intakeMotorBrakeMode;
    topConfigs.Slot0.kP = TurretConstants.kTopP;
    topConfigs.Slot0.kV = TurretConstants.kTopV;
    topConfigs.Slot0.kS = TurretConstants.kTopS;
    topConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    topConfigs.CurrentLimits.StatorCurrentLimit = TurretConstants.topCurrentLimit;
    positionMotor.getConfigurator().apply(topConfigs);
    // Apply to signals
    poositionDeg = positionMotor.getPosition();
    // Set polling frequency and optimizations
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocityRPS);
    positionMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocityRPS, poositionDeg);

    inputs.velocityRPM = velocityRPS.getValue().in(Rotation.per(Minute));
    inputs.positionDeg = poositionDeg.getValue().in(Degrees);
    inputs.positionConnected = positionMotor.isConnected();
  }

  public void setPosition(double position) {
    requestPosition.withPosition(position);
  }

  public void holdPosition(){
     setPosition(getPosition());
  }

  public double getPosition() {
    return Rotation2d.fromRotations(positionMotor.getPosition().getValueAsDouble()).getDegrees();
  }

  @Override
  public void stop() {
    positionMotor.stopMotor();
  }
}