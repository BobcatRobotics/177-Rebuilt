package frc.robot.subsystems.Hopper;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import org.bobcatrobotics.Hardware.Characterization.CharacterizationClosedLoopOutputType;
import org.bobcatrobotics.Util.CANDeviceDetails;
import org.bobcatrobotics.Util.CANDeviceDetails.Hardware;
import org.bobcatrobotics.Util.CANDeviceDetails.Manufacturer;
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
import frc.robot.RobotState;
import frc.robot.subsystems.Hopper.Modules.ModuleConfigurator;

public class HopperSim implements HopperIO {

  private  TalonFX hopperTopMotor;
  private SimMotorFX hopperTopMotorSim;
  public ModuleConfigurator hopperConfigTop;

  private final VelocityTorqueCurrentFOC topRequestVelocity = new VelocityTorqueCurrentFOC(0);

  private StatusSignal<AngularVelocity> velocityOfHopperTopRPS;
  private StatusSignal<Current> statorCurrentOfHopperTopAmps;
  private StatusSignal<Voltage> outputOfHopperTopVolts;
  private StatusSignal<AngularAcceleration> accelerationOfHopperTop;

  private TorqueCurrentFOC characterizationRequestTorqueCurrentFOC = new TorqueCurrentFOC(0);
  private VoltageOut characterizationRequestVoltage = new VoltageOut(0);


  private TunablePID hopperTopPID;
  private TunablePID hopperBottomPID;

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
  }

  public void setupTopMotor(Gains g) {
    hopperTopPID = new TunablePID(
        "/Hopper/Top/PID", g);
    hopperConfigTop = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.HopperConstants.Top.hopperMotorId,
        Constants.HopperConstants.Top.isInverted,
        Constants.HopperConstants.Top.isCoast,
        Constants.HopperConstants.Top.hopperCurrentLimit);
    hopperTopMotor = new TalonFX(hopperConfigTop.getMotorInnerId(), new CANBus("rio"));
    hopperConfigTop.configureMotor(hopperTopMotor, hopperTopPID);
    velocityOfHopperTopRPS = hopperTopMotor.getVelocity();
    statorCurrentOfHopperTopAmps = hopperTopMotor.getStatorCurrent();
    outputOfHopperTopVolts = hopperTopMotor.getMotorVoltage();
    accelerationOfHopperTop = hopperTopMotor.getAcceleration();
    hopperConfigTop.configureSignals(hopperTopMotor, 50.0, velocityOfHopperTopRPS,
        statorCurrentOfHopperTopAmps, accelerationOfHopperTop, accelerationOfHopperTop);
    
      CANDeviceDetails tmp = new CANDeviceDetails(hopperConfigTop.getMotorId(),"rio",Hardware.TalonFX,Manufacturer.Ctre,"Hopper");
    List<CANDeviceDetails> rioDevices = RobotState.getInstance().devices.get("rio");
    rioDevices.add(tmp);
    RobotState.getInstance().devices.replace("rio", rioDevices);
  }



  @Override
  public void updateInputs(HopperIOInputs inputs) {
    hopperTopMotorSim.update();
    hopperTopMotor = hopperTopMotorSim.apply(hopperTopMotor);

    BaseStatusSignal.refreshAll(velocityOfHopperTopRPS,
        statorCurrentOfHopperTopAmps, accelerationOfHopperTop, outputOfHopperTopVolts);

    inputs.velocityOfHopperTopRPS = hopperTopMotorSim.getVelocity();
    inputs.statorCurrentOfHopperTopAmps = hopperTopMotorSim.getCurrent();
      inputs.accelerationOfHopperTop = hopperTopMotorSim.getAcceleration();
    inputs.outputOfHopperTopVolts = hopperTopMotorSim.getVoltage();
    inputs.hopperTopConnected = hopperTopMotor.isConnected();
  }


    public  void setVelocity(HopperState desiredState) {
    setVelocity(desiredState.getHopperSpeedOfTop());
  }


  public void setVelocity(double topVelocity){
    setTopSpeed(topVelocity);
  }

  public void setTopSpeed(double speed){
    hopperTopMotor.setControl(topRequestVelocity.withVelocity(speed));
  }


  public void stopTop(){
    hopperSetpointTop = 0.0;
    hopperTopMotor.stopMotor();
  }

  @Override
  public void stop() {
    stopTop();
  }

  @Override
  public void periodic() {
    if (hopperTopPID.hasChanged()) {
      hopperConfigTop.updateMotorPID(hopperTopMotor, hopperTopPID);
    }
  }

    /* Characterization */
  public void runCharacterization_Hopper(double output) {
    hopperTopMotor.setControl(switch (CharacterizationClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity_Hopper() {
    double avg = (
        hopperTopMotor.getVelocity().getValue().in(RotationsPerSecond)) / 1;
    return avg;
  }
}