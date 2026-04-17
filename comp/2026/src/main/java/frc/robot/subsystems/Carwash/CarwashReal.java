package frc.robot.subsystems.Carwash;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import org.bobcatrobotics.Hardware.Characterization.CharacterizationClosedLoopOutputType;
import org.bobcatrobotics.Util.CANDeviceDetails;
import org.bobcatrobotics.Util.CANDeviceDetails.Manufacturer;
import org.bobcatrobotics.Util.Tunables.Gains;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Shooter.Modules.ModuleConfigurator;

public class CarwashReal implements CarwashIO {
  private TalonFX shooterIntakeMotor;
  public ModuleConfigurator intakeWheelConfig;
  private VelocityTorqueCurrentFOC velIntakeRequest = new VelocityTorqueCurrentFOC(0);
  private TorqueCurrentFOC characterizationRequestTorqueCurrentFOC = new TorqueCurrentFOC(0);

  private VoltageOut characterizationRequestVoltage = new VoltageOut(0);
  private StatusSignal<AngularVelocity> velocityOfIntakeRPS;
  private StatusSignal<Current> statorCurrentOfIntakeAmps;
  private StatusSignal<Voltage> outputOfIntakeVolts;
  private StatusSignal<AngularAcceleration> accelerationOfIntake;
  public double intakeSetpoint = 0;

  public CarwashReal() {
    Gains intakeGains = new Gains.Builder()
        .kP(Constants.CarwashConstants.SharedIntake.kIntakeMotorkP)
        .kI(Constants.CarwashConstants.SharedIntake.kIntakeMotorkI)
        .kD(Constants.CarwashConstants.SharedIntake.kIntakeMotorkD)
        .kS(Constants.CarwashConstants.SharedIntake.kIntakeMotorkS)
        .kV(Constants.CarwashConstants.SharedIntake.kIntakeMotorkV)
        .kA(Constants.CarwashConstants.SharedIntake.kIntakeMotorkA).build();

    setupIntake(intakeGains);
  }

  public void setupIntake(Gains g) {
    intakeWheelConfig = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.CarwashConstants.SharedIntake.intakeIDLeft,
        Constants.CarwashConstants.SharedIntake.isInverted,
        Constants.CarwashConstants.SharedIntake.isCoast,
        Constants.CarwashConstants.SharedIntake.statorCurrentLimit,
        Constants.CarwashConstants.SharedIntake.supplyCurrentLimit);
    shooterIntakeMotor = new TalonFX(intakeWheelConfig.getMotorInnerId(), new CANBus("rio"));
    intakeWheelConfig.configureMotor(shooterIntakeMotor, g);
    if (Constants.lowTelemetryMode) {
      velocityOfIntakeRPS = shooterIntakeMotor.getVelocity();
      statorCurrentOfIntakeAmps = shooterIntakeMotor.getStatorCurrent();
      intakeWheelConfig.configureSignals(shooterIntakeMotor, 50.0, velocityOfIntakeRPS,
          statorCurrentOfIntakeAmps);
    } else {
      velocityOfIntakeRPS = shooterIntakeMotor.getVelocity();
      statorCurrentOfIntakeAmps = shooterIntakeMotor.getStatorCurrent();
      outputOfIntakeVolts = shooterIntakeMotor.getMotorVoltage();
      accelerationOfIntake = shooterIntakeMotor.getAcceleration();
      intakeWheelConfig.configureSignals(shooterIntakeMotor, 50.0, velocityOfIntakeRPS,
          statorCurrentOfIntakeAmps, outputOfIntakeVolts, accelerationOfIntake);
    }
    CANDeviceDetails tmp = new CANDeviceDetails(intakeWheelConfig.getMotorId(),"rio",Manufacturer.Ctre,"Carwash");
    List<CANDeviceDetails> rioDevices = RobotState.getInstance().devices.get("rio");
    rioDevices.add(tmp);
    RobotState.getInstance().devices.replace("rio", rioDevices);
  }

  public void updateInputs(CarwashIOInputs inputs) {
    if (Constants.lowTelemetryMode) {
      lowTelemetry(inputs);
    } else {
      highTelemetry(inputs);
    }
  }

  public void highTelemetry(CarwashIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        accelerationOfIntake,
        outputOfIntakeVolts);
    inputs.accelerationOfIntake = accelerationOfIntake.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.outputOfIntakeVolts = outputOfIntakeVolts.getValue().in(Volts);
    lowTelemetry(inputs);
  }

  public void lowTelemetry(CarwashIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocityOfIntakeRPS, statorCurrentOfIntakeAmps);
    inputs.velocityOfIntakeRPS = velocityOfIntakeRPS.getValue().in(Rotations.per(Seconds));
    inputs.statorCurrentOfIntakeAmps = statorCurrentOfIntakeAmps.getValue().in(Amps);
    inputs.shooterIntakeMotorConnected = shooterIntakeMotor.isConnected();
    inputs.torqueCurrentCarwashAmps = shooterIntakeMotor.getTorqueCurrent().getValue().in(Amps);
  }

  public void setVelocity(CarwashState desiredState) {
    setVelocity(desiredState.getIntakeSpeed());
  }

  public void setVelocity(double shooterIntakeSpeed) {
    setIntakeSpeed(shooterIntakeSpeed);
  }

  public void reverseCarwash(double shooterIntakeSpeed) {
    setIntakeSpeed(-shooterIntakeSpeed);
  }

  public void setIntakeSpeed(double shooterIntakeSpeedInRPS) {
    intakeSetpoint = shooterIntakeSpeedInRPS;
    shooterIntakeMotor.setControl(velIntakeRequest.withVelocity(intakeSetpoint));
  }

  public void holdPosition() {
  }

  public void stopIntakeMotor() {
    intakeSetpoint = 0;
    shooterIntakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
  }

  public void simulationPeriodic() {
  }

  /* Characterization */
  public void runCharacterization_Intake(double output) {
    shooterIntakeMotor.setControl(switch (CharacterizationClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity_Intake() {
    double avg = shooterIntakeMotor.getVelocity().getValue().in(RotationsPerSecond);
    return avg;
  }
}