package frc.robot.subsystems.intake;

import frc.robot.Constants;

public class IntakeState {
  public class ShooterGoal {
    public double position;
    public double speed;
  }

  public enum State {
    IDLE,
    MANUAL,
    TARGETTING
  }

  public State currentState = State.IDLE;
  private double manualSpeed = 0.0;

  /** Set the intake to a predefined state */
  public void setState(State state) {
    this.currentState = state;
  }

  /** Set manual speed and switch to MANUAL mode */
  public void setManualSpeed(double speed) {
    manualSpeed = speed;
    currentState = State.MANUAL;
  }

  /** Returns the motor output based on the current state */
  public ShooterGoal getOutput() {
    ShooterGoal goal = new ShooterGoal();
    switch (currentState) {
      case IDLE -> {
        goal.speed = Constants.IntakeConstants.idleRollerSpeed;
      }
      case MANUAL -> {
        goal.speed = manualSpeed;
      }
      case TARGETTING -> {
        goal.speed = targetSpeed;
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
