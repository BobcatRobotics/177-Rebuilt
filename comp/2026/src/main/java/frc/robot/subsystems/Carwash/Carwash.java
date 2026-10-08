package frc.robot.subsystems.Carwash;

import java.util.function.DoubleSupplier;

import org.bobcatrobotics.Hardware.Characterization.SysIdModule;
import org.bobcatrobotics.Hardware.Characterization.SysIdRegistry;
import org.bobcatrobotics.Util.Interpolators.TripleOutputInterpolator;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Carwash.CarwashState.CarwashGoal;
import frc.robot.subsystems.Carwash.CarwashState.State;

public class Carwash extends SubsystemBase {

  private final CarwashIO io;
  private final CarwashIOInputsAutoLogged inputs = new CarwashIOInputsAutoLogged();

  private CarwashState desiredState;
  private final SysIdRegistry sysIdRegistry = new SysIdRegistry();
  

  public Carwash(CarwashIO io) {
    // Configure SysId

    SysIdRoutine.Config intakeSysIdconfig = new SysIdRoutine.Config(
        null, // ramp rate
        null, // step voltage
        null, // timeout
        state -> Logger.recordOutput("Carwash/Intake/SysIdState", state.toString()));

    sysIdRegistry.register("SysIdStateIntake", new SysIdModule(
        "Carwash/SysIdStateIntake",
        this,
        this::runCharacterization_Intake, intakeSysIdconfig));

    this.io = io;

  }

  public void applyState() {
    desiredState = new CarwashState();
    desiredState.setState(State.IDLE);
  }

  @Override
  public void periodic() {
    desiredState.update();
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Carwash/inputs", inputs);
    Logger.recordOutput("Carwash/State", desiredState.getCurrentState());

  }

  public void setState(CarwashState state) {
    desiredState = state;
    setVelocity(desiredState.getCurrentState());
  }

  private void setVelocity(CarwashState.State state) {
    desiredState.setState(state);
    io.setVelocity(desiredState);
  }

  private void setVelocity(
      double shooterIntakeSpeed) {
    io.setVelocity( shooterIntakeSpeed);
  }

  public void reverseCarwash(double shooterOuttakeSpeed) {
    io.setVelocity(shooterOuttakeSpeed);
  }
  

  public void setIntakeSpeed(double shooterIntakeSpeed) {
    io.setIntakeSpeed(shooterIntakeSpeed);
  }

  public void holdPosition() {
    io.holdPosition();
  }

  public void stop() {
    io.stop();
  }

  public void stopIntakeWheel() {
    io.stopIntakeWheel();
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }


  public void runCharacterization_Intake(double output) {
    io.runCharacterization_Intake(output);
  }

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native
   * units).
   */
  public double getFFCharacterizationVelocity_Intake() {
    double output = io.getFFCharacterizationVelocity_Intake();
    return output;
  }

  public SysIdRegistry getRegistry() {
    return sysIdRegistry;
  }

  public void spinUp() {
    RobotState.getInstance().getCarwashState().setState(CarwashState.State.TARGETING);
    CarwashGoal goal = new CarwashGoal();
    goal.intakeSpeed = -13;
    RobotState.getInstance().getCarwashState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getCarwashState());
  }

  

  public void feedFuel() {
    RobotState.getInstance().getCarwashState().setState(CarwashState.State.INTERPOLATING);
    CarwashGoal goal = new CarwashGoal();
    goal.intakeSpeed = RobotState.getInstance().getCarwashState().getIntakeSpeed();
    RobotState.getInstance().getCarwashState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getCarwashState());
  }

  public void manualFeedFuel() {
    RobotState.getInstance().getCarwashState().setState(CarwashState.State.TARGETING);
    CarwashGoal goal = new CarwashGoal();
    goal.intakeSpeed = Constants.CarwashConstants.targetIntakeSpeedRPS;
    RobotState.getInstance().getCarwashState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getCarwashState());
  }



  public void stopFeedingFuel() {
    RobotState.getInstance().getCarwashState().setState(CarwashState.State.TARGETING);
    CarwashGoal goal = new CarwashGoal();
    goal.intakeSpeed = 0;
    RobotState.getInstance().getCarwashState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getCarwashState());
  }

  public void reverseFuel() {
    RobotState.getInstance().getCarwashState().setState(CarwashState.State.TARGETING);
    CarwashGoal goal = new CarwashGoal();
    goal.intakeSpeed = Constants.CarwashConstants.targetIntakeSpeedRPS*-1;
    RobotState.getInstance().getCarwashState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getCarwashState());
  }

  public void reverseFuel( DoubleSupplier carwashrps) {
    RobotState.getInstance().getCarwashState().setState(CarwashState.State.TARGETING);
    CarwashGoal goal = new CarwashGoal();
    goal.intakeSpeed = carwashrps.getAsDouble() * -1;
    RobotState.getInstance().getCarwashState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getCarwashState());
  }



  /**
   * True when the carwash is at or above its setpoint, i.e. no fuel is loading it. The auto shoot
   * commands use this (debounced) to tell when the hopper is empty.
   *
   * One-sided on purpose: with an 80 RPS setpoint, an empty carwash runs about 84.5-91 RPS
   * depending on battery, so a +/- band around any one value misses some batteries and the auto
   * shot never ends. A fed carwash dips to about 40-77 RPS.
   */
  public boolean atSpeed() {
    // getVelocityCarwash() is NaN when stopped or reversed, which compares false
    boolean isAtTolerance =
        getVelocityCarwash() >= RobotState.getInstance().getCarwashState().getIntakeSpeed();
    Logger.recordOutput("Carwash/isUpToSpeed", isAtTolerance);
    return isAtTolerance;
  }

  public double getVelocityCarwash() {
    int count = 0;
    double avg = 0;
    if (inputs.velocityOfIntakeRPS > 0) {
      avg += inputs.velocityOfIntakeRPS;
      count += 1;
    }
    return avg / count;
  }
}