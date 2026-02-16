package frc.robot.subystems.Hopper;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.bobcatrobotics.Hardware.Characterization.CharacterizationClosedLoopOutputType;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

import frc.robot.Constants.HopperConstants;
import frc.robot.subystems.Hopper.Modules.ModuleConfigurator;
import org.bobcatrobotics.Util.Tunables.Gains;
import org.bobcatrobotics.Util.Tunables.TunablePID;

public class HopperSim implements HopperIO {

  private  TalonFX hopperTopMotor;
  private SimMotorFX hopperTopMotorSim;
  public ModuleConfigurator hopperConfigTop;
  private  TalonFX hopperBottomMotor;
  private SimMotorFX hopperBottomMotorSim;
  public ModuleConfigurator hopperConfigBottom;

  private final VelocityTorqueCurrentFOC topRequestVelocity = new VelocityTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC bottomRequestVelocity = new VelocityTorqueCurrentFOC(0);

  private StatusSignal<AngularVelocity> velocityOfHopperTopRPS;
  private StatusSignal<Current> statorCurrentOfHopperTopAmps;
  private StatusSignal<Voltage> outputOfHopperTopVolts;
  private StatusSignal<AngularAcceleration> accelerationOfHopperTop;

  private StatusSignal<AngularVelocity> velocityOfHopperBottomRPS;
  private StatusSignal<Current> statorCurrentOfHopperBottomAmps;
  private StatusSignal<Voltage> outputOfHopperBottomVolts;
  private StatusSignal<AngularAcceleration> accelerationOfHopperBottom;

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
    Gains bottomMotorGains = new Gains.Builder()
        .kP(Constants.HopperConstants.Bottom.kHopperP)
        .kI(Constants.HopperConstants.Bottom.kHopperI)
        .kD(Constants.HopperConstants.Bottom.kHopperD)
        .kS(Constants.HopperConstants.Bottom.kHopperS)
        .kV(Constants.HopperConstants.Bottom.kHopperV)
        .kA(Constants.HopperConstants.Bottom.kHopperA).build();
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
  }

  public void setupBottomMotor(Gains g) {
    hopperBottomPID = new TunablePID(
        "/Hopper/Bottom/PID", g);
    hopperConfigBottom = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.HopperConstants.Bottom.hopperMotorId,
        Constants.HopperConstants.Bottom.isInverted,
        Constants.HopperConstants.Bottom.isCoast,
        Constants.HopperConstants.Bottom.hopperCurrentLimit);
    hopperBottomMotor = new TalonFX(hopperConfigTop.getMotorInnerId(), new CANBus("rio"));
    hopperConfigBottom.configureMotor(hopperBottomMotor, hopperBottomPID);
    velocityOfHopperBottomRPS = hopperBottomMotor.getVelocity();
    statorCurrentOfHopperBottomAmps = hopperBottomMotor.getStatorCurrent();
    outputOfHopperBottomVolts = hopperBottomMotor.getMotorVoltage();
    accelerationOfHopperBottom = hopperBottomMotor.getAcceleration();
    hopperConfigBottom.configureSignals(hopperBottomMotor, 50.0, velocityOfHopperBottomRPS,
        statorCurrentOfHopperBottomAmps, outputOfHopperBottomVolts, accelerationOfHopperBottom);
  }


  @Override
  public void updateInputs(HopperIOInputs inputs) {
    hopperBottomMotorSim.update();
    hopperBottomMotor = hopperBottomMotorSim.apply(hopperBottomMotor);
    hopperTopMotorSim.update();
    hopperTopMotor = hopperTopMotorSim.apply(hopperTopMotor);

    BaseStatusSignal.refreshAll(velocityOfHopperTopRPS,
        statorCurrentOfHopperTopAmps, accelerationOfHopperTop, outputOfHopperTopVolts,
        velocityOfHopperBottomRPS,
        statorCurrentOfHopperBottomAmps, outputOfHopperBottomVolts, accelerationOfHopperBottom);

    inputs.velocityOfHopperTopRPS = hopperBottomMotorSim.getVelocity();
    inputs.statorCurrentOfHopperTopAmps = hopperBottomMotorSim.getCurrent();
      inputs.accelerationOfHopperTop = hopperBottomMotorSim.getAcceleration();
    inputs.outputOfHopperTopVolts = hopperBottomMotorSim.getVoltage();
    inputs.hopperTopConnected = hopperTopMotor.isConnected();

        inputs.velocityOfHopperBottomRPS = hopperTopMotorSim.getVelocity();
    inputs.statorCurrentOfHopperBottomAmps = hopperTopMotorSim.getCurrent();
      inputs.accelerationOfHopperBottom = hopperTopMotorSim.getAcceleration();
    inputs.outputOfHopperBottomVolts = hopperTopMotorSim.getVoltage();
    inputs.hopperBottomConnected = hopperBottomMotor.isConnected();
  }


    public  void setVelocity(HopperState desiredState) {
    setVelocity(desiredState.getHopperSpeedOfTop(),
        desiredState.getHopperSpeedOfBottom());
  }


  public void setVelocity(double topVelocity, double bottomVelocity){
    setTopSpeed(topVelocity);
    setBottomSpeed(bottomVelocity);
  }

  public void setTopSpeed(double speed){
    hopperTopMotor.setControl(topRequestVelocity.withVelocity(speed));
  }

  public void setBottomSpeed(double speed){
      hopperBottomMotor.setControl(bottomRequestVelocity.withVelocity(speed));
  }



  public void stopTop(){
    hopperSetpointTop = 0.0;
    hopperTopMotor.stopMotor();
  }
  public void stopBottom(){
    hopperSetpointBottom = 0.0;
    hopperBottomMotor.stopMotor();
  }
  @Override
  public void stop() {
    stopTop();
    stopBottom();
  }

  @Override
  public void periodic() {
    if (hopperBottomPID.hasChanged()) {
      hopperConfigBottom.updateMotorPID(hopperBottomMotor, hopperBottomPID);
    }
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
    hopperBottomMotor.setControl(switch (CharacterizationClosedLoopOutputType.Voltage) {
      case Voltage -> characterizationRequestVoltage.withOutput(output);
      case TorqueCurrentFOC -> characterizationRequestTorqueCurrentFOC.withOutput(output);
    });
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity_Hopper() {
    double avg = (
        hopperTopMotor.getVelocity().getValue().in(RotationsPerSecond) +
        hopperBottomMotor.getVelocity().getValue().in(RotationsPerSecond)) / 2;
    return avg;
  }
}