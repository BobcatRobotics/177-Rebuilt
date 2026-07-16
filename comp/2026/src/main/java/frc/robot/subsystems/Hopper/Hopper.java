package frc.robot.subsystems.Hopper;

import org.bobcatrobotics.Hardware.Characterization.SysIdModule;
import org.bobcatrobotics.Hardware.Characterization.SysIdRegistry;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.subsystems.Hopper.HopperState.HopperGoal;
import frc.robot.subsystems.Hopper.HopperState.State;


public class Hopper extends SubsystemBase {

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  private HopperState desiredState;
  private final SysIdRegistry sysIdRegistry = new SysIdRegistry();

  public Hopper(HopperIO io) {
    // Configure SysId

    SysIdRoutine.Config HopperSysIdconfig = new SysIdRoutine.Config(
        null, // ramp rate
        null, // step voltage
        null, // timeout
        state -> Logger.recordOutput("Hopper/SysIdState", state.toString()));

    sysIdRegistry.register("SysIdStateHopper", new SysIdModule(
        "Hopper/SysIdStateHopper",
        this,
        this::runCharacterization_Hopper, HopperSysIdconfig));

    this.io = io;
  }

  public void applyState() {
    desiredState = new HopperState();
    desiredState.setState(State.IDLE);
  }

  @Override
  public void periodic() {
    desiredState.update();
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Hopper/inputs", inputs);
    Logger.recordOutput("Hopper/State", desiredState.getCurrentState());
  }

  public void setState(HopperState state) {
    desiredState = state;
    setVelocity(desiredState.getCurrentState());
  }

  private void setVelocity(HopperState.State state) {
    desiredState.setState(state);
    io.setVelocity(desiredState);
  }

  private void setVelocity(double topVelocity) {
    io.setVelocity(topVelocity);
  }


  public void setTopVelocity(double topVelocity) {
    io.setTopSpeed(topVelocity);
  }

  public void holdPosition() {
    io.holdPosition();
  }

  public void stop() {
    io.stop();
  }


  public void stopTopWheel() {
    io.stopTop();

  }


  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }


  public void runCharacterization_Hopper(double output) {
    io.runCharacterization_Hopper(output);
  }

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native
   * units).
   */
  public double getFFCharacterizationVelocity_Hopper() {
    double output = io.getFFCharacterizationVelocity_Hopper();
    return output;
  }
  
  public SysIdRegistry getRegistry() {
    return sysIdRegistry;
  }

  public void runHopper(){
    RobotState.getInstance().getHopperState().setState(HopperState.State.TARGETING);
    HopperGoal goal = new HopperGoal();
    goal.hopperSpeedTop = RobotState.getInstance().getHopperState().getHopperSpeedOfTop();
    RobotState.getInstance().getHopperState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getHopperState());
  }
    public void reverseHopper(){
    RobotState.getInstance().getHopperState().setState(HopperState.State.TARGETING);
    HopperGoal goal = new HopperGoal();
    goal.hopperSpeedTop = 50 * -1;
    RobotState.getInstance().getHopperState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getHopperState());
  }

   public void hopperSpinUp(){
    RobotState.getInstance().getHopperState().setState(HopperState.State.TARGETING);
    HopperGoal goal = new HopperGoal();
    goal.hopperSpeedTop = -30;
    RobotState.getInstance().getHopperState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getHopperState());
  }

}