package frc.robot.subsystems.Carwash;

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
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;

import edu.wpi.first.math.controller.PIDController;
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
import org.bobcatrobotics.Util.Tunables.Gains;
import org.bobcatrobotics.Util.Tunables.TunablePID;
import org.bobcatrobotics.Hardware.Motors.Sim.PidContants;
import org.bobcatrobotics.Hardware.Motors.Sim.SimMotor;
import org.bobcatrobotics.Hardware.Motors.Sim.SimMotorVel;

public class CarwashSim implements CarwashIO {
  private TalonFX shooterIntakeMotor;
  private SimMotor shooterIntakeMotorSim;
  
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

  
    setupIntake(intakeGains);

    shooterIntakeMotorSim = new SimMotorVel(shooterIntakeMotor, Constants.CarwashConstants.SharedIntake.isInverted,new PidContants(5,Constants.CarwashConstants.SharedIntake.kIntakeMotorkS),new PIDController(300, 0,0.01)) ;
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
    shooterIntakeMotorSim.setVelocity(shooterIntakeSpeedInRPS);
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
    shooterIntakeMotorSim.update();
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