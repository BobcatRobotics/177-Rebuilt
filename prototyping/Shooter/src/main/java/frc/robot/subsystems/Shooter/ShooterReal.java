package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants.ShooterConstants;;

public class ShooterReal implements ShooterIO {

  private final TalonFX bottomMotorLeft = new TalonFX(17);
  private final TalonFX bottomMotorRight = new TalonFX(22);

  private final TalonFX TopMotorBottomWheel = new TalonFX(30);
  private final TalonFX TopMotorTopWheel = new TalonFX(2);

  private final VelocityTorqueCurrentFOC requestVelocity = new VelocityTorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC requestPosition = new PositionTorqueCurrentFOC(0);
  private StatusSignal<AngularVelocity> velocityRightRPS;
  private StatusSignal<AngularVelocity> velocityLeftRPS;
  private StatusSignal<AngularVelocity> velocityTopTopWheelRPS;
  private StatusSignal<AngularVelocity> velocityTopBottomWheelRPS;

  private StatusSignal<Angle> poositionDeg;
  private Alert LicenseTopWheelAlert = new Alert(
      "License Motor Top TOp WHeel is Not Found",
      AlertType.kError);
  private Alert LicenseBottomWheelAlert = new Alert(
      "License Motor Top Bottom Wheel is Not Found",
      AlertType.kError);

  private Alert LicenseRightAlert = new Alert(
      "License Motor Bottom Right is Not Found",
      AlertType.kError);
  private Alert LicenseLeftAlert = new Alert(
      "License Motor Bottom Left is Not Found",
      AlertType.kError);

  public ShooterReal() {
    // Top motor configurations
    TalonFXConfiguration shooterLeftConfig = new TalonFXConfiguration();
    bottomMotorLeft.getConfigurator().apply(shooterLeftConfig); // reset to default
    shooterLeftConfig.MotorOutput.Inverted = ShooterConstants.bottomMotorLeftInvert;
    shooterLeftConfig.MotorOutput.NeutralMode = ShooterConstants.bottomMotorLeftBrakeMode;
    shooterLeftConfig.Slot0.kP = ShooterConstants.kBottomLeftP;
    shooterLeftConfig.Slot0.kV = ShooterConstants.kBottomLeftV;
    shooterLeftConfig.Slot0.kS = ShooterConstants.kBottomLeftS;
    shooterLeftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterLeftConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.bottomLeftCurrentLimit;
    bottomMotorLeft.getConfigurator().apply(shooterLeftConfig);
    // Bottom motor configurations
    TalonFXConfiguration bottomConfigs = new TalonFXConfiguration();
    bottomMotorRight.getConfigurator().apply(bottomConfigs); // reset to default
    bottomConfigs.MotorOutput.Inverted = ShooterConstants.bottomRightMotorInvert;
    bottomConfigs.MotorOutput.NeutralMode = ShooterConstants.bottomRightMotorBrakeMode;
    bottomConfigs.Slot0.kP = ShooterConstants.kBottomRightP;
    bottomConfigs.Slot0.kV = ShooterConstants.kBottomRightV;
    bottomConfigs.Slot0.kS = ShooterConstants.kBottomRightS;
    bottomConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    bottomConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.bottomRightCurrentLimit;
    bottomMotorRight.getConfigurator().apply(bottomConfigs);

    // Top motor configurations
    TalonFXConfiguration topWheelConfigs = new TalonFXConfiguration();
    TopMotorTopWheel.getConfigurator().apply(topWheelConfigs); // reset to default
    topWheelConfigs.MotorOutput.Inverted = ShooterConstants.topTopMotorInvert;
    topWheelConfigs.MotorOutput.NeutralMode = ShooterConstants.topTopMotorBrakeMode;
    topWheelConfigs.Slot0.kP = ShooterConstants.kTopTopP;
    topWheelConfigs.Slot0.kV = ShooterConstants.kTopTopV;
    topWheelConfigs.Slot0.kS = ShooterConstants.kTopTopS;
    topWheelConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    topWheelConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.topTopCurrentLimit;
    TopMotorTopWheel.getConfigurator().apply(topWheelConfigs);
    // Bottom motor configurations
    TalonFXConfiguration TopBottomConfigs = new TalonFXConfiguration();
    TopMotorBottomWheel.getConfigurator().apply(TopBottomConfigs); // reset to default
    TopBottomConfigs.MotorOutput.Inverted = ShooterConstants.topBottomMotorInvert;
    TopBottomConfigs.MotorOutput.NeutralMode = ShooterConstants.topBottomMotorBrakeMode;
    TopBottomConfigs.Slot0.kP = ShooterConstants.kTopBottomP;
    TopBottomConfigs.Slot0.kV = ShooterConstants.kTopBottomS;
    TopBottomConfigs.Slot0.kS = ShooterConstants.kTopBottomV;
    TopBottomConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    TopBottomConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.topBottomCurrentLimit;
    TopMotorBottomWheel.getConfigurator().apply(TopBottomConfigs);

    // Apply to signals
    velocityRightRPS = bottomMotorRight.getVelocity();
    velocityLeftRPS = bottomMotorLeft.getVelocity();
    velocityTopTopWheelRPS = TopMotorTopWheel.getVelocity();
    velocityTopBottomWheelRPS = TopMotorBottomWheel.getVelocity();
    // Set polling frequency and optimizations
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocityRightRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocityLeftRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocityTopTopWheelRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocityTopBottomWheelRPS);
    bottomMotorLeft.optimizeBusUtilization();
    bottomMotorRight.optimizeBusUtilization();
    TopMotorBottomWheel.optimizeBusUtilization();
    TopMotorTopWheel.optimizeBusUtilization();

    LicenseTopWheelAlert.set(TopMotorTopWheel.getIsProLicensed().getValue());
    LicenseBottomWheelAlert.set(TopMotorBottomWheel.getIsProLicensed().getValue());
    Logger.recordOutput("Motor/TopTopWheel/Licensed", TopMotorTopWheel.getIsProLicensed().getValue());
    Logger.recordOutput("Motor/TopBottomWheel/Licensed", TopMotorBottomWheel.getIsProLicensed().getValue());

    LicenseLeftAlert.set(bottomMotorLeft.getIsProLicensed().getValue());
    LicenseRightAlert.set(bottomMotorRight.getIsProLicensed().getValue());
    Logger.recordOutput("Motor/BottomLeft/Licensed", bottomMotorLeft.getIsProLicensed().getValue());
    Logger.recordOutput("Motor/BottomRight/Licensed", bottomMotorRight.getIsProLicensed().getValue());
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocityLeftRPS, velocityRightRPS);

    inputs.velocityLeftRPS = velocityLeftRPS.getValue().in(Rotation.per(Seconds));

    inputs.velocityRightRPS = velocityRightRPS.getValue().in(Rotation.per(Seconds));

    inputs.velocityTopWheelRPS = velocityTopTopWheelRPS.getValue().in(Rotation.per(Seconds));

    inputs.velocityBottomWheelRPS = velocityTopBottomWheelRPS.getValue().in(Rotation.per(Seconds));
  }

  public void setOutput(double shooterOutput, double backspinOutput, double angleOutput) {
    bottomMotorLeft.set(shooterOutput);
    bottomMotorRight.set(shooterOutput);
    
    TopMotorTopWheel.set(backspinOutput);
    TopMotorBottomWheel.set(angleOutput);
  }

  public void setPosition(double position) {
    requestPosition.withPosition(position);
  }

  public void setVelocity(double velocity) {
    bottomMotorLeft.setControl(requestVelocity.withVelocity(velocity));
    bottomMotorRight.setControl(requestVelocity.withVelocity(velocity));

    TopMotorBottomWheel.setControl(requestVelocity.withVelocity(velocity));
    TopMotorTopWheel.setControl(requestVelocity.withVelocity(velocity));
  }

  public void holdPosition() {
    setPosition(getPosition());
  }

  public double getPosition() {
    return Rotation2d.fromRotations(bottomMotorLeft.getPosition().getValueAsDouble()).getDegrees();
  }

  public double getVelocity() {
    return bottomMotorLeft.getVelocity().getValueAsDouble();
  }

  @Override
  public void stop() {
    bottomMotorLeft.stopMotor();
    bottomMotorRight.stopMotor();
    TopMotorBottomWheel.stopMotor();
    TopMotorTopWheel.stopMotor();
  }
}