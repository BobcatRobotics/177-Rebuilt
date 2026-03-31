package frc.robot.subsystems.Hopper;


import org.bobcatrobotics.Util.Tunables.TunableDouble;

import frc.robot.Constants;

public class HopperState {

  /** Output goal for the shooter subsystem */
  public static class HopperGoal {
    public double hopperSpeedTop;
  }

  public enum State {
    IDLE,
    TARGETING
  }

  private State currentState = State.IDLE;
  private HopperGoal currentSetpoints = new HopperGoal();


  // Manual control values



  public HopperState() {

  }

  /** Set the shooter to a predefined state */
  public void setState(State state) {
    this.currentState = state;
  }

  /**
   * Set all shooter speeds at once and switch to MANUAL mode
   */


   /** Returns the shooter outputs based on the current state */
  public void update() {

    switch (currentState) {
      case IDLE -> {
         currentSetpoints.hopperSpeedTop = Constants.HopperConstants.idleHopperSpeed;
      }
      case TARGETING -> {
        // Placeholder – typically filled in by vision / interpolation
        currentSetpoints.hopperSpeedTop = Constants.HopperConstants.topMotorTargetVelocity;
      }
    }
  }

  public void setCurrentSetPoints(HopperGoal goal){
    currentSetpoints = goal;
  }

  public State getCurrentState() {
    return currentState;
  }


  public double getHopperSpeedOfTop() {
    return currentSetpoints.hopperSpeedTop;
  }
}
