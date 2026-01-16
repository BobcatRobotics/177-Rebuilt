package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.Constants.ShooterConstants;;

public class ShooterReal implements ShooterIO {

  private final TalonFX topMotor = new TalonFX(10);
  private final TalonFX bottomMotor = new TalonFX(11);
  private final TalonFX kickerMotor = new TalonFX(12);

  private final VelocityTorqueCurrentFOC requestTop = new VelocityTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC requestBottom = new VelocityTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC requestKicker = new VelocityTorqueCurrentFOC(0);

    private StatusSignal<Current> topMotorStatorCurrent;
    private StatusSignal<AngularVelocity> topMotorVelocityRPS;
    private StatusSignal<Current> bottomMotorStatorCurrent;
    private StatusSignal<AngularVelocity> bottomMotorVelocityRPS;
    private StatusSignal<Current> kickerMotorStatorCurrent;
    private StatusSignal<AngularVelocity> kickerMotorVelocityRPS;

  public ShooterReal() {
    // Top motor configurations
    TalonFXConfiguration topConfigs = new TalonFXConfiguration();
    topMotor.getConfigurator().apply(topConfigs); // reset to default
    topConfigs.MotorOutput.Inverted = ShooterConstants.topMotorInvert;
    topConfigs.MotorOutput.NeutralMode = ShooterConstants.topMotorBrakeMode;
    topConfigs.Slot0.kP = ShooterConstants.kTopP;
    topConfigs.Slot0.kV = ShooterConstants.kTopV;
    topConfigs.Slot0.kS = ShooterConstants.kTopS;
    topConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    topConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.topCurrentLimit;
    topMotor.getConfigurator().apply(topConfigs);
    // Bottom motor configurations
    TalonFXConfiguration bottomConfigs = new TalonFXConfiguration();
    bottomMotor.getConfigurator().apply(bottomConfigs); // reset to default
    bottomConfigs.MotorOutput.Inverted = ShooterConstants.bottomMotorInvert;
    bottomConfigs.MotorOutput.NeutralMode = ShooterConstants.bottomMotorBrakeMode;
    bottomConfigs.Slot0.kP = ShooterConstants.kBottomP;
    bottomConfigs.Slot0.kV = ShooterConstants.kBottomV;
    bottomConfigs.Slot0.kS = ShooterConstants.kBottomS;
    bottomConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    bottomConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.bottomCurrentLimit;
    bottomMotor.getConfigurator().apply(bottomConfigs);
    // Kicker motor configurations
    TalonFXConfiguration kickerConfigs = new TalonFXConfiguration();
    bottomMotor.getConfigurator().apply(kickerConfigs); // reset to default
    kickerConfigs.MotorOutput.Inverted = ShooterConstants.kickerMotorInvert;
    kickerConfigs.MotorOutput.NeutralMode = ShooterConstants.kickerMotorBrakeMode;
    kickerConfigs.Slot0.kP = ShooterConstants.kKickerP;
    kickerConfigs.Slot0.kV = ShooterConstants.kKickerV;
    kickerConfigs.Slot0.kS = ShooterConstants.kKickerS;
    kickerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    kickerConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.kickerCurrentLimit;
    kickerMotor.getConfigurator().apply(kickerConfigs);
    // Apply to signals
    
    topMotorStatorCurrent = topMotor.getStatorCurrent();
    topMotorVelocityRPS = topMotor.getVelocity();
    bottomMotorStatorCurrent = bottomMotor.getStatorCurrent();
    bottomMotorVelocityRPS = bottomMotor.getVelocity();
    kickerMotorStatorCurrent = kickerMotor.getStatorCurrent();
    kickerMotorVelocityRPS = kickerMotor.getVelocity();
    // Set polling frequency and optimizations
    BaseStatusSignal.setUpdateFrequencyForAll(50, topMotorStatorCurrent, topMotorVelocityRPS, bottomMotorStatorCurrent, bottomMotorVelocityRPS,kickerMotorStatorCurrent, kickerMotorVelocityRPS);
    topMotor.optimizeBusUtilization();
    bottomMotor.optimizeBusUtilization();
    kickerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        topMotorStatorCurrent,
        topMotorVelocityRPS,
        bottomMotorStatorCurrent,
        bottomMotorVelocityRPS,
        kickerMotorStatorCurrent,
        kickerMotorVelocityRPS);

    inputs.topVelocityRPM = topMotor.getVelocity().getValueAsDouble() * 60.0;
    inputs.bottomVelocityRPM = bottomMotor.getVelocity().getValueAsDouble() * 60.0;
    inputs.kickerVelocityRPM = kickerMotor.getVelocity().getValueAsDouble() * 60.0;

    inputs.topAppliedVolts = topMotor.getMotorVoltage().getValueAsDouble();
    inputs.bottomAppliedVolts = bottomMotor.getMotorVoltage().getValueAsDouble();
    inputs.kickerAppliedVolts = kickerMotor.getMotorVoltage().getValueAsDouble();

    inputs.topConnected = topMotor.isConnected();
    inputs.bottomConnected = bottomMotor.isConnected();
    inputs.kickerConnected = kickerMotor.isConnected();
  }

  @Override
  public void setTopVelocity(double rpm) {
    topMotor.setControl(requestTop.withVelocity(rpm / 60.0));
  }

  @Override
  public void setBottomVelocity(double rpm) {
    bottomMotor.setControl(requestBottom.withVelocity(rpm / 60.0));
  }

  @Override
  public void setKickerVelocity(double rpm) {
    kickerMotor.setControl(requestKicker.withVelocity(rpm / 60.0));
  }

  @Override
  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
    kickerMotor.stopMotor();
  }
}