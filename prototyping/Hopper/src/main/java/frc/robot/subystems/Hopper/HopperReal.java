package frc.robot.subystems.Hopper;

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

import frc.robot.Constants.HopperConstants;

public class HopperReal implements HopperIO {

  private final TalonFX hopperTopMotor = new TalonFX(11);
  private final TalonFX hopperBottomMotor = new TalonFX(12);

  private final VelocityTorqueCurrentFOC topRequestVelocity = new VelocityTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC bottomRequestVelocity = new VelocityTorqueCurrentFOC(0);

  private StatusSignal<AngularVelocity> topVelocityRPS;
  private StatusSignal<AngularVelocity> bottomVelocityRPS;

  public HopperReal() {
    // motor configurations both use same configuration change if wrong
    TalonFXConfiguration hopperBottomMotorConfigs = new TalonFXConfiguration();
    hopperBottomMotor.getConfigurator().apply(hopperBottomMotorConfigs); // reset to default
    hopperBottomMotorConfigs.MotorOutput.Inverted = HopperConstants.hopperMotorInvert;
    hopperBottomMotorConfigs.MotorOutput.NeutralMode = HopperConstants.hopperMotorBrakeMode;
    hopperBottomMotorConfigs.Slot0.kP = HopperConstants.kTopP;
    hopperBottomMotorConfigs.Slot0.kV = HopperConstants.kTopV;
    hopperBottomMotorConfigs.Slot0.kS = HopperConstants.kTopS;
    hopperBottomMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    hopperBottomMotorConfigs.CurrentLimits.StatorCurrentLimit = HopperConstants.hopperCurrentLimit;
    hopperTopMotor.getConfigurator().apply(hopperBottomMotorConfigs);

//invert the top motor
   TalonFXConfiguration hopperTopMotorConfigs = new TalonFXConfiguration();
    hopperBottomMotor.getConfigurator().apply(hopperTopMotorConfigs); // reset to default
    hopperTopMotorConfigs.MotorOutput.Inverted = HopperConstants.hopperMotorInvert;
    hopperTopMotorConfigs.MotorOutput.NeutralMode = HopperConstants.hopperMotorBrakeMode;
    hopperTopMotorConfigs.Slot0.kP = HopperConstants.kTopP;
    hopperTopMotorConfigs.Slot0.kV = HopperConstants.kTopV;
    hopperTopMotorConfigs.Slot0.kS = HopperConstants.kTopS;
    hopperTopMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    hopperTopMotorConfigs.CurrentLimits.StatorCurrentLimit = HopperConstants.hopperCurrentLimit;
    hopperTopMotor.getConfigurator().apply(hopperTopMotorConfigs);
    // Apply to signals
    topVelocityRPS = hopperTopMotor.getVelocity();
    bottomVelocityRPS = hopperBottomMotor.getVelocity();
    // Set polling frequency and optimizations
    BaseStatusSignal.setUpdateFrequencyForAll(50, topVelocityRPS);
    BaseStatusSignal.setUpdateFrequencyForAll(50, bottomVelocityRPS);
    hopperTopMotor.optimizeBusUtilization();
    hopperBottomMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        topVelocityRPS, bottomVelocityRPS);

    inputs.topVelocityRPM = topVelocityRPS.getValue().in(Rotation.per(Minute));
    inputs.bottomVelocityRPM = bottomVelocityRPS.getValue().in(Rotation.per(Minute));
    inputs.topMotorConnected = hopperTopMotor.isConnected();
    inputs.bottomMotorConnected = hopperBottomMotor.isConnected();
  }


  public void setTopVelocity(double velocity) {
    topRequestVelocity.withVelocity(velocity);
  }

  public void setBottomVelocity(double velocity) {
    bottomRequestVelocity.withVelocity(velocity);
  }

  public double getTopVelocity() {
    return hopperTopMotor.getVelocity().getValueAsDouble();
  }

  public double getBottomVelocity() {
    return hopperBottomMotor.getVelocity().getValueAsDouble();
  }


  @Override
  public void stop() {
    hopperTopMotor.stopMotor();
    hopperBottomMotor.stopMotor();
  }
}