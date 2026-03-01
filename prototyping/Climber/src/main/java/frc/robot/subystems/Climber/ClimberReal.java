package frc.robot.subystems.Climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.io.ObjectInputFilter.Status;

import org.bobcatrobotics.Hardware.Characterization.CharacterizationClosedLoopOutputType;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subystems.Climber.Modules.ModuleConfigurator;
import org.bobcatrobotics.Util.Tunables.Gains;
import org.bobcatrobotics.Util.Tunables.TunablePID;

public class ClimberReal implements ClimberIO {

  private TalonFX climberMotor;
  public ModuleConfigurator climberConfig;

  private final PositionTorqueCurrentFOC climberRequestPosition = new PositionTorqueCurrentFOC(null);

  private StatusSignal<AngularVelocity> velocityOfClimberRPS;
  private StatusSignal<Current> statorCurrentOfClimberAmps;
  private StatusSignal<Voltage> outputOfClimberVolts;
  private StatusSignal<AngularAcceleration> accelerationOfClimber;
  private StatusSignal<Angle> climberPosition;

  private PositionTorqueCurrentFOC requestPosition = new PositionTorqueCurrentFOC(0);

  private TunablePID climberPID;

  public double climberSetpoint = 0;

  public ClimberReal() {

    // Flywheel Configuration
    Gains climberMotorGains = new Gains.Builder()
        .kP(Constants.ClimberConstants.kClimberP)
        .kI(Constants.ClimberConstants.kClimberI)
        .kD(Constants.ClimberConstants.kClimberD)
        .kS(Constants.ClimberConstants.kClimberS)
        .kV(Constants.ClimberConstants.kClimberV)
        .kA(Constants.ClimberConstants.kClimberA).build();

    setupClimberMotor(climberMotorGains);
  }

  public void setupClimberMotor(Gains g) {
    climberPID = new TunablePID(
        "/Climber/PID", g);
    climberConfig = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.ClimberConstants.climberMotorId,
        Constants.ClimberConstants.isInverted,
        Constants.ClimberConstants.isCoast,
        Constants.ClimberConstants.climberCurrentLimit);
    climberMotor = new TalonFX(climberConfig.getMotorInnerId(), new CANBus("rio"));
    climberConfig.configureMotor(climberMotor, climberPID);
    velocityOfClimberRPS = climberMotor.getVelocity();
    statorCurrentOfClimberAmps = climberMotor.getStatorCurrent();
    outputOfClimberVolts = climberMotor.getMotorVoltage();
    accelerationOfClimber = climberMotor.getAcceleration();
    climberPosition = climberMotor.getPosition();
    climberConfig.configureSignals(climberMotor, 50.0, velocityOfClimberRPS,
        statorCurrentOfClimberAmps, outputOfClimberVolts, accelerationOfClimber, climberPosition);
  }

  

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocityOfClimberRPS,
        statorCurrentOfClimberAmps, accelerationOfClimber, outputOfClimberVolts
        );

    inputs.velocityOfClimberRPS = velocityOfClimberRPS.getValue().in(Rotation.per(Minute));
    inputs.statorCurrentOfClimberAmps = statorCurrentOfClimberAmps.getValue().in(Amps);
      inputs.accelerationOfClimber = accelerationOfClimber.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.outputOfClimberVolts = outputOfClimberVolts.getValue().in(Volts);
    inputs.climberConnected = climberMotor.isConnected();
    inputs.climberPosition = climberMotor.getPosition().getValueAsDouble();
  }


  public void setPosition(ClimberState desiredState) {
    setPosition(desiredState.getClimberPosition());
  }

  public void setPosition(double pos) {
    requestPosition.withPosition(pos);
  }


  public void stopMotor(){
    climberMotor.stopMotor();
  }
  @Override
  public void stop() {
    stopMotor();
  }

  @Override
  public void periodic() {
    if (climberPID.hasChanged()) {
      climberConfig.updateMotorPID(climberMotor, climberPID);
    }
  }

    /* Characterization */
  // public void runCharacterization_Climber(double output) {
  //   climberMotor.setControl(switch (CharacterizationClosedLoopOutputType.PositionTorqueCurrentFOC) {
  //     case Voltage -> characterizationRequestVoltage.withOutput(output);
  //     case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
  //   });}

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity_Climber() {
        return climberMotor.getVelocity().getValue().in(RotationsPerSecond);
  }
}