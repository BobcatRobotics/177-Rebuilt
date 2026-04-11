package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.bobcatrobotics.Hardware.Characterization.CharacterizationClosedLoopOutputType;
import org.bobcatrobotics.Util.CANDeviceDetails;
import org.bobcatrobotics.Util.CANDeviceDetails.Hardware;
import org.bobcatrobotics.Util.CANDeviceDetails.Manufacturer;
import org.bobcatrobotics.Util.Tunables.Gains;
import org.littletonrobotics.junction.Logger;

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
import frc.robot.subsystems.Hopper.Modules.ModuleConfigurator;

public class HopperRealSingle implements HopperIO {

  private TalonFX hopperMotor;
  public ModuleConfigurator hopperConfig;

  private final VelocityTorqueCurrentFOC topRequestVelocity = new VelocityTorqueCurrentFOC(0);

  private StatusSignal<AngularVelocity> velocityOfHopperTopRPS;
  private StatusSignal<Current> statorCurrentOfHopperTopAmps;
  private StatusSignal<Voltage> outputOfHopperTopVolts;
  private StatusSignal<AngularAcceleration> accelerationOfHopperTop;

  private TorqueCurrentFOC characterizationRequestTorqueCurrentFOC = new TorqueCurrentFOC(0);
  private VoltageOut characterizationRequestVoltage = new VoltageOut(0);


  public double hopperSetpoint = 0;



  public HopperRealSingle() {

    // Flywheel Configuration
    Gains topMotorGains = new Gains.Builder()
        .kP(Constants.HopperConstants.Top.kHopperP)
        .kI(Constants.HopperConstants.Top.kHopperI)
        .kD(Constants.HopperConstants.Top.kHopperD)
        .kS(Constants.HopperConstants.Top.kHopperS)
        .kV(Constants.HopperConstants.Top.kHopperV)
        .kA(Constants.HopperConstants.Top.kHopperA).build();

    setupTopMotor(topMotorGains);
  }


    public void updateCanDetails(String bus, int id, Hardware hardware, Manufacturer manufacturer , String subsystemname, boolean status) {
      CANDeviceDetails carwashDeviceDetails = new CANDeviceDetails(id,bus,hardware,manufacturer,subsystemname,status);
      List<CANDeviceDetails> rioDevices = RobotState.getInstance().devices.get(bus);
      rioDevices.add(carwashDeviceDetails);
      RobotState.getInstance().devices.replace(bus, rioDevices);
      RobotState.getInstance().subsytemHopperDevices.add(carwashDeviceDetails);
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

  public void setupTopMotor(Gains g) {
    hopperConfig = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.HopperConstants.Top.hopperMotorId,
        Constants.HopperConstants.Top.isInverted,
        Constants.HopperConstants.Top.isCoast,
        Constants.HopperConstants.Top.hopperCurrentLimit);
    hopperMotor = new TalonFX(hopperConfig.getMotorInnerId(), new CANBus("rio"));
    hopperConfig.configureMotor(hopperMotor, g);
    if (Constants.lowTelemetryMode) {
      velocityOfHopperTopRPS = hopperMotor.getVelocity();
      statorCurrentOfHopperTopAmps = hopperMotor.getStatorCurrent();
      hopperConfig.configureSignals(hopperMotor, 50.0, velocityOfHopperTopRPS,
          statorCurrentOfHopperTopAmps);
    } else {
      velocityOfHopperTopRPS = hopperMotor.getVelocity();
      statorCurrentOfHopperTopAmps = hopperMotor.getStatorCurrent();
      outputOfHopperTopVolts = hopperMotor.getMotorVoltage();
      accelerationOfHopperTop = hopperMotor.getAcceleration();
      hopperConfig.configureSignals(hopperMotor, 50.0, velocityOfHopperTopRPS,
          statorCurrentOfHopperTopAmps, accelerationOfHopperTop, accelerationOfHopperTop);
    }
    updateCanDetails("rio",hopperConfig.getMotorInnerId(),Hardware.TalonFX,Manufacturer.Ctre,"Hopper",hopperMotor.isConnected());
    }


  @Override
  public void updateInputs(HopperIOInputs inputs) {
    if (Constants.lowTelemetryMode) {
      lowTelemetry(inputs);
    } else {
      highTelemetry(inputs);
    }

        updateCanDetails("rio",hopperConfig.getMotorInnerId(),Hardware.TalonFX,hopperMotor.isConnected());
  }

  public void highTelemetry(HopperIOInputs inputs) {
    BaseStatusSignal.refreshAll( accelerationOfHopperTop, outputOfHopperTopVolts);
    inputs.accelerationOfHopperTop = accelerationOfHopperTop.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.outputOfHopperTopVolts = outputOfHopperTopVolts.getValue().in(Volts);
    lowTelemetry(inputs);
  }

  public void lowTelemetry(HopperIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocityOfHopperTopRPS,statorCurrentOfHopperTopAmps);
    inputs.velocityOfHopperTopRPS = velocityOfHopperTopRPS.getValue().in(Rotation.per(Minute));
    inputs.statorCurrentOfHopperTopAmps = statorCurrentOfHopperTopAmps.getValue().in(Amps);
    inputs.hopperTopConnected = hopperMotor.isConnected();
  }

  public void setVelocity(HopperState desiredState) {
    setVelocity(desiredState.getHopperSpeedOfTop());
  }

  public void setVelocity(double topVelocity) {
    setTopSpeed(topVelocity);
  }

  public void setTopSpeed(double speed) {
    hopperSetpoint = speed;
    
    Logger.recordOutput("/Hopper/SetPointSpeed",hopperSetpoint);
    hopperMotor.setControl(topRequestVelocity.withVelocity(hopperSetpoint));
  }

  public void stopTop() {
    hopperSetpoint = 0.0;
    Logger.recordOutput("/Hopper/SetPointSpeed",hopperSetpoint);
    hopperMotor.stopMotor();
  }


  @Override
  public void stop() {
    stopTop();
  }

  @Override
  public void periodic() {
  }

  /* Characterization */
  public void runCharacterization_Hopper(double output) {
    hopperMotor.setControl(switch (CharacterizationClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity_Hopper() {
    double avg = (hopperMotor.getVelocity().getValue().in(RotationsPerSecond)) / 1;
    return avg;
  }

}