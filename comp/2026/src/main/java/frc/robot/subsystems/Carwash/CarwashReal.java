package frc.robot.subsystems.Carwash;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.bobcatrobotics.Hardware.Characterization.CharacterizationClosedLoopOutputType;
import org.bobcatrobotics.Util.CANDeviceDetails;
import org.bobcatrobotics.Util.CANDeviceDetails.Hardware;
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

        updateCanDetails("rio",intakeWheelConfig.getMotorInnerId(),Hardware.TalonFX,Manufacturer.Ctre,"Carwash",shooterIntakeMotor.isConnected());
    }

  
    public void updateCanDetails(String bus, int id, Hardware hardware, Manufacturer manufacturer , String subsystemname, boolean status) {
      CANDeviceDetails carwashDeviceDetails = new CANDeviceDetails(id,bus,hardware,manufacturer,subsystemname,status);
      List<CANDeviceDetails> rioDevices = RobotState.getInstance().devices.get(bus);
      rioDevices.add(carwashDeviceDetails);
      RobotState.getInstance().devices.replace(bus, rioDevices);
      RobotState.getInstance().subsytemCarwashDevices.add(carwashDeviceDetails);
    }


  public void updateCanDetails(String bus, int id, Hardware hardware, boolean status) {
    List<CANDeviceDetails> fulldevices = RobotState.getInstance().devices.get(bus);

    Optional<CANDeviceDetails> opt = fulldevices.stream()
        .filter(obj -> obj.id() == id && obj.hardware().equals(hardware))
        .findFirst();

    if (opt.isEmpty()) {
        return; // or handle error/log
    }

    CANDeviceDetails old = opt.get();

    CANDeviceDetails updated = new CANDeviceDetails(
        old.id(),
        old.bus(),
        old.hardware(),
        old.manufacturer(),
        old.subsystemName(),
        status
    );

    fulldevices.replaceAll(item ->
        (item.id() == id && item.hardware().equals(hardware)) ? updated : item
    );

    RobotState.getInstance().devices.put(bus, fulldevices);
}


  public void updateInputs(CarwashIOInputs inputs) {
    if (Constants.lowTelemetryMode) {
      lowTelemetry(inputs);
    } else {
      highTelemetry(inputs);
    }

     updateCanDetails("rio",intakeWheelConfig.getMotorInnerId(),Hardware.TalonFX,shooterIntakeMotor.isConnected());
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