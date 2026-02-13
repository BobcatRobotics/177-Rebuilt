package frc.robot.subsystems.Intake;

import frc.robot.Constants;

public class IntakeState {
  public class IntakeGoal {
    public double position;
    public double speed;
  }

  public enum State {
    IDLE,
    DEPLOYED
  }

  public State currentState = State.IDLE;
  private double manualSpeed = 0.0; //Placeholder, might need to make a constant

  /** Set the intake to a predefined state */
  public void setState(State state) {
    this.currentState = state;
  }


  /** Returns the motor output based on the current state */
  public IntakeGoal getOutput() {
    IntakeGoal goal = new IntakeGoal();
    switch (currentState) {
      case IDLE -> {
        goal.speed = Constants.IntakeConstants.idleRollerSpeed;
      }
      case DEPLOYED -> {
        goal.speed = manualSpeed;
      }
    }
    return goal;
  }

  public State getCurrentState() {
    return currentState;
  }

  public double getSpeed() {
    return manualSpeed;
  }
}
