package frc.robot.subystems.Climber;


import org.bobcatrobotics.Util.Tunables.TunableDouble;

import frc.robot.Constants;

public class ClimberState {

  /** Output goal for the shooter subsystem */
  public static class ClimberGoal {
    public double climberPosition;
  }

  public enum State {
    IDLE,
    MANUAL,
    TARGETING
  
  }

  private State currentState = State.IDLE;
  private ClimberGoal currentSetpoints = new ClimberGoal();


  // Manual control values
  private TunableDouble manualClimberPosition;


  public ClimberState() {
       manualClimberPosition = new TunableDouble("/Climber/manualPosition",0.0);
  }

  /** Set the Climber to a predefined state */
  public void setState(State state) {
    this.currentState = state;
  }

  /**
   * Set Climber position and switch to MANUAL mode
   */
  public void setManualPosition(
      double climberPosition) {
       manualClimberPosition = new TunableDouble("/Climber/positionSetpoint",
          climberPosition);
      currentState = State.MANUAL;
  }

   /** Returns the shooter outputs based on the current state */
  public void update() {

    switch (currentState) {
      case IDLE -> {
        currentSetpoints.climberPosition = Constants.ClimberConstants.idleClimberPosition;
      }
      case MANUAL -> {
        currentSetpoints.climberPosition = Constants.ClimberConstants.manualClimberPosition;
      }
      case TARGETING -> {
        currentSetpoints.climberPosition = Constants.ClimberConstants.targetClimberPosition;
      }
    }
  }

   public void setCurrentSetPoints(ClimberGoal goal) {
    currentSetpoints = goal;
  }

  public State getCurrentState() {
    return currentState;
  }


  public double getClimberPosition() {
    return currentSetpoints.climberPosition;
  }
}
