package frc.robot.subystems.Hopper;


import org.bobcatrobotics.Util.Tunables.TunableDouble;

import frc.robot.Constants;

public class HopperState {

  /** Output goal for the shooter subsystem */
  public static class HopperGoal {
    public double hopperSpeedTop;
    public double hopperSpeedBottom;
  }

  public enum State {
    IDLE,
    MANUAL,
    TARGETING
  }

  private State currentState = State.IDLE;
  private HopperGoal currentSetpoints = new HopperGoal();


  // Manual control values
  private TunableDouble manualHopperSpeedTop;
  private TunableDouble manualHopperSpeedBottom;


  public HopperState() {
       manualHopperSpeedTop = new TunableDouble("/Hopper/Top/manualBackspinSetPointTop", 0.0);
      manualHopperSpeedBottom = new TunableDouble("/Hopper/Bottom/manualBackspinSetPointBottom", 0.0);

  }

  /** Set the shooter to a predefined state */
  public void setState(State state) {
    this.currentState = state;
  }

  /**
   * Set all shooter speeds at once and switch to MANUAL mode
   */
  public void setManualSpeeds(
      double hopperSpeedTop,
      double hopperSpeedBottom) {
       manualHopperSpeedTop = new TunableDouble("/Hopper/Top/manualBackspinSetPointTop",
          hopperSpeedTop);
      manualHopperSpeedBottom = new TunableDouble("/Hopper/Bottom/manualBackspinSetPointBottom",
          hopperSpeedBottom);
    currentState = State.MANUAL;
  }

   /** Returns the shooter outputs based on the current state */
  public void update() {

    switch (currentState) {
      case IDLE -> {
         currentSetpoints.hopperSpeedTop = Constants.HopperConstants.idleHopperSpeed;
        currentSetpoints.hopperSpeedBottom = Constants.HopperConstants.idleHopperSpeed;
      }
      case MANUAL -> {
          currentSetpoints.hopperSpeedTop = manualHopperSpeedTop.get();
        currentSetpoints.hopperSpeedBottom = manualHopperSpeedBottom.get();
      }

      case TARGETING -> {
        // Placeholder – typically filled in by vision / interpolation
        currentSetpoints.hopperSpeedTop = Constants.HopperConstants.topMotorTargetVelocity;
        currentSetpoints.hopperSpeedBottom = Constants.HopperConstants.bottomMotorTargetVelocity;
      }
    }
  }

  public State getCurrentState() {
    return currentState;
  }


  public double getHopperSpeedOfTop() {
    return currentSetpoints.hopperSpeedTop;
  }
  public double getHopperSpeedOfBottom() {
    return currentSetpoints.hopperSpeedBottom;
  }
}
