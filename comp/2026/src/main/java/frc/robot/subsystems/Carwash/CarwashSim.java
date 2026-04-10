package frc.robot.subsystems.Carwash;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import org.bobcatrobotics.Hardware.Characterization.CharacterizationClosedLoopOutputType;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Shooter.Modules.ModuleConfigurator;

import org.bobcatrobotics.Util.CANDeviceDetails;
import org.bobcatrobotics.Util.CANDeviceDetails.Manufacturer;
import org.bobcatrobotics.Util.Tunables.Gains;
import org.bobcatrobotics.Util.Tunables.TunablePID;

public class CarwashSim implements CarwashIO {
  private TalonFX shooterIntakeMotor;
  private  TalonFXSimState shooterIntakeMotorState;
  private FlywheelSim m_motorSimModel
  
  ;
  public ModuleConfigurator intakeWheelConfig;

  // Defines tunable values , particularly for configurations of motors ( IE PIDs
  // )
  private VelocityTorqueCurrentFOC velIntakeRequest = new VelocityTorqueCurrentFOC(0);

  private TorqueCurrentFOC characterizationRequestTorqueCurrentFOC = new TorqueCurrentFOC(0);
  private VoltageOut characterizationRequestVoltage = new VoltageOut(0);

  private StatusSignal<AngularVelocity> velocityOfIntakeRPS;
  private StatusSignal<Current> statorCurrentOfIntakeAmps;
  private StatusSignal<Voltage> outputOfIntakeVolts;
  private StatusSignal<AngularAcceleration> accelerationOfIntake;

  public double intakeSetpoint = 0;
  private TunablePID intakePID;

  public CarwashSim() {
    Gains intakeGains = new Gains.Builder()
        .kP(Constants.CarwashConstants.SharedIntake.kIntakeMotorkP)
        .kI(Constants.CarwashConstants.SharedIntake.kIntakeMotorkI)
        .kD(Constants.CarwashConstants.SharedIntake.kIntakeMotorkD)
        .kS(Constants.CarwashConstants.SharedIntake.kIntakeMotorkS)
        .kV(Constants.CarwashConstants.SharedIntake.kIntakeMotorkV)
        .kA(Constants.CarwashConstants.SharedIntake.kIntakeMotorkA).build();

    
    DCMotor motorModel = DCMotor.getKrakenX60Foc(1);
    m_motorSimModel = new FlywheelSim(LinearSystemId.createFlywheelSystem(motorModel, .00003, 1),motorModel);

    setupIntake(intakeGains);
    shooterIntakeMotorState = shooterIntakeMotor.getSimState();


  }

  public void setupIntake(Gains g) {
    // Intake Configuration
    intakePID = new TunablePID(
        "/Shooter/Intake/PID", g);
    intakeWheelConfig = new ModuleConfigurator(g.toSlot0Configs(),
        Constants.CarwashConstants.SharedIntake.intakeIDLeft,
        Constants.CarwashConstants.SharedIntake.isInverted,
        Constants.CarwashConstants.SharedIntake.isCoast,
        Constants.CarwashConstants.SharedIntake.statorCurrentLimit,
        Constants.CarwashConstants.SharedIntake.supplyCurrentLimit);
    shooterIntakeMotor = new TalonFX(intakeWheelConfig.getMotorInnerId(), new CANBus("rio"));
    intakeWheelConfig.configureMotor(shooterIntakeMotor, intakePID);
    velocityOfIntakeRPS = shooterIntakeMotor.getVelocity();
    statorCurrentOfIntakeAmps = shooterIntakeMotor.getStatorCurrent();
    outputOfIntakeVolts = shooterIntakeMotor.getMotorVoltage();
    accelerationOfIntake = shooterIntakeMotor.getAcceleration();
    intakeWheelConfig.configureSignals(shooterIntakeMotor, 50.0, velocityOfIntakeRPS,
        statorCurrentOfIntakeAmps, outputOfIntakeVolts, accelerationOfIntake);
    CANDeviceDetails tmp = new CANDeviceDetails(intakeWheelConfig.getMotorId(),"rio",Manufacturer.Ctre,"Carwash");
    List<CANDeviceDetails> rioDevices = RobotState.getInstance().devices.get("rio");
    rioDevices.add(tmp);
    RobotState.getInstance().devices.replace("rio", rioDevices);
    }

  public void updateInputs(CarwashIOInputs inputs) {

    BaseStatusSignal.refreshAll(
        velocityOfIntakeRPS,
        accelerationOfIntake,
        outputOfIntakeVolts);

    inputs.velocityOfIntakeRPS = velocityOfIntakeRPS.getValue().in(Rotations.per(Seconds));
    inputs.accelerationOfIntake = accelerationOfIntake.getValue()
        .in(RotationsPerSecondPerSecond);
    inputs.statorCurrentOfIntakeAmps = statorCurrentOfIntakeAmps.getValue().in(Amps);

    inputs.shooterIntakeMotorConnected = shooterIntakeMotor.isConnected();
    inputs.outputOfIntakeVolts = outputOfIntakeVolts.getValue().in(Volts);

  }

  public void setVelocity(CarwashState desiredState) {
    setVelocity(desiredState.getIntakeSpeed());
  }

  public void setVelocity( double shooterIntakeSpeed) {
    setIntakeSpeed(shooterIntakeSpeed);
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
    // Get  Motor Output Voltage
    shooterIntakeMotorState = shooterIntakeMotor.getSimState();
    double motorVoltage = shooterIntakeMotorState.getMotorVoltage();
    // Feed Into Physics Simulation
    m_motorSimModel.setInputVoltage(motorVoltage);
    // Udpate SIM ( 20ms loop )
    m_motorSimModel.update(0.02);
    // get voltage ( rad/sec -> rotations/sec)
    double velocityRadPerSec = m_motorSimModel.getAngularVelocityRadPerSec();
    double velocityRotPerSec = velocityRadPerSec / (2*Math.PI);
    // PUSH intop the TalonFX simulated Sensor the value
    shooterIntakeMotorState.setRotorVelocity(-velocityRotPerSec);
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
    double avg = shooterIntakeMotor.getVelocity().getValueAsDouble();
    return avg;
  }
}