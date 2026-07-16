package frc.robot.subsystems.Turret;

import frc.robot.Constants;

public class TurretState {
  public class TurretGoal {
    public double position;
    public double speed;
  }

  public enum State {
    IDLE,
    MANUAL,
    TARGETTING
  }

  public State currentState = State.IDLE;
  private double manualPosition = 0.0;

  /** Set the intake to a predefined state */
  public void setState(State state) {
    this.currentState = state;
  }


  public void setManualPosition(double position) {
    manualPosition = position;
    currentState = State.MANUAL;
  }

  /** Returns the motor output based on the current state */
  public TurretGoal getOutput() {
    TurretGoal goal = new TurretGoal();
    switch (currentState) {
      case IDLE -> {
        goal.position = Constants.TurretConstants.zeroRollerPosition;
      }
      case MANUAL -> {
        goal.position = manualPosition;
      }
    }
    return goal;
  }

  public State getCurrentState() {
    return currentState;
  }

  public double getPosition(){
    return manualPosition;
  }
}
