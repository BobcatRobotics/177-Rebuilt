package frc.robot.subystems.Climber;

import org.bobcatrobotics.Hardware.Characterization.SysIdModule;
import org.bobcatrobotics.Hardware.Characterization.SysIdRegistry;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subystems.Climber.ClimberState.ClimberGoal;
import frc.robot.subystems.Climber.ClimberState.State;
import frc.robot.RobotState;


public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  

  private ClimberState desiredState;
  private final SysIdRegistry sysIdRegistry = new SysIdRegistry();

  public Climber(ClimberIO io) {
    // Configure SysId

    SysIdRoutine.Config ClimberSysIdconfig = new SysIdRoutine.Config(
        null, // ramp rate
        null, // step voltage
        null, // timeout
        state -> Logger.recordOutput("Climber/SysIdState", state.toString()));

    sysIdRegistry.register("SysIdStateClimber", new SysIdModule(
        "Climber/SysIdStateClimber",
        this,
        this::runCharacterization_Climber, ClimberSysIdconfig));

    this.io = io;
  }

  public void applyState() {
    desiredState = new ClimberState();
    desiredState.setState(State.IDLE);
  }

  @Override
  public void periodic() {
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Climber/inputs", inputs);
    Logger.recordOutput("Climber/State", desiredState.getCurrentState());
  }

  public void setState(ClimberState state) {
    desiredState = state;
    setPosition(desiredState.getCurrentState());
  }

  private void setPosition(ClimberState.State state) {
    desiredState.setState(state);
    io.setPosition(desiredState);
  }

  public void holdPosition() {
    io.holdPosition();
  }

  public void stop() {
    io.stop();
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }


  public void runCharacterization_Climber(double output) {
    io.runCharacterization_Climber(output);
  }

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native
   * units).
   */
  public double getFFCharacterizationVelocity_Climber() {
    double output = io.getFFCharacterizationVelocity_Climber();
    return output;
  }

  public void deployClimber(){
      RobotState.getInstance().getClimberState().setState(ClimberState.State.MANUAL);
      ClimberGoal goal = new ClimberGoal();
      //Should be radians
      goal.climberPosition = 1.5;
      RobotState.getInstance().getClimberState().setCurrentSetPoints(goal);
      setState(RobotState.getInstance().getClimberState());
  }
  
  public SysIdRegistry getRegistry() {
    return sysIdRegistry;
  }


}