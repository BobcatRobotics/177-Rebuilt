package frc.robot.subsystems.Hopper;


import org.bobcatrobotics.Util.Tunables.TunableDouble;

import frc.robot.Constants;

public class HopperState {

  /** Output goal for the shooter subsystem */
  public static class ShooterGoal {
    public double hopperSpeedTop;
    public double hopperSpeedBottom;
  }

  public enum State {
    IDLE,
    MANUAL,
    TARGETING
  }

  private State currentState = State.IDLE;


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
  public ShooterGoal getOutput() {
    ShooterGoal goal = new ShooterGoal();

    switch (currentState) {
      case IDLE -> {
         goal.hopperSpeedTop = Constants.HopperConstants.idleHopperSpeed;
        goal.hopperSpeedBottom = Constants.HopperConstants.idleHopperSpeed;
      }
      case MANUAL -> {
          goal.hopperSpeedTop = manualHopperSpeedTop.get();
        goal.hopperSpeedBottom = manualHopperSpeedBottom.get();
      }

      case TARGETING -> {
        // Placeholder – typically filled in by vision / interpolation
        goal.hopperSpeedTop = Constants.HopperConstants.topMotorTargetVelocity;
        goal.hopperSpeedBottom = Constants.HopperConstants.bottomMotorTargetVelocity;
      }
    }

    return goal;
  }

  public State getCurrentState() {
    return currentState;
  }


  public double getHopperSpeedOfTop() {
    return getOutput().hopperSpeedTop;
  }
  public double getHopperSpeedOfBottom() {
    return getOutput().hopperSpeedBottom;
  }
}
