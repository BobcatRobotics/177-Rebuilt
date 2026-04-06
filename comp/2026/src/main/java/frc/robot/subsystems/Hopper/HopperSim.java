package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.bobcatrobotics.Hardware.Characterization.CharacterizationClosedLoopOutputType;
import org.bobcatrobotics.Util.Tunables.Gains;
import org.bobcatrobotics.Util.Tunables.TunablePID;

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
import frc.robot.subsystems.Hopper.Modules.ModuleConfigurator;

public class HopperSim implements HopperIO {

  private TalonFX hopperMotor;
  public ModuleConfigurator hopperConfig;
  private SimMotorFX hopperTopMotorSim;

  private final VelocityTorqueCurrentFOC topRequestVelocity = new VelocityTorqueCurrentFOC(0);

  private StatusSignal<AngularVelocity> velocityOfHopperTopRPS;
  private StatusSignal<Current> statorCurrentOfHopperTopAmps;
  private StatusSignal<Voltage> outputOfHopperTopVolts;
  private StatusSignal<AngularAcceleration> accelerationOfHopperTop;

  private TorqueCurrentFOC characterizationRequestTorqueCurrentFOC = new TorqueCurrentFOC(0);
  private VoltageOut characterizationRequestVoltage = new VoltageOut(0);

  public double hopperSetpointTop = 0;
  public double hopperSetpointBottom = 0;

  public HopperSim() {

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
        hopperTopMotorSim = new SimMotorFX(hopperMotor, Constants.HopperConstants.Top.isInverted);
  }



  @Override
  public void updateInputs(HopperIOInputs inputs) {
    hopperTopMotorSim.update();
    if (Constants.lowTelemetryMode) {
      lowTelemetry(inputs);
    } else {
      highTelemetry(inputs);
    }
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


    public  void setVelocity(HopperState desiredState) {
    setVelocity(desiredState.getHopperSpeedOfTop());
  }


  public void setVelocity(double topVelocity){
    setTopSpeed(topVelocity);
  }

  public void setTopSpeed(double speed){
    hopperMotor.setControl(topRequestVelocity.withVelocity(speed));
    hopperTopMotorSim.setVelocity(speed);
  }


  public void stopTop(){
    hopperSetpointTop = 0.0;
    hopperMotor.stopMotor();
    hopperTopMotorSim.setVelocity(0);
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
    double avg = (
        hopperMotor.getVelocity().getValue().in(RotationsPerSecond)) / 1;
    return avg;
  }
}