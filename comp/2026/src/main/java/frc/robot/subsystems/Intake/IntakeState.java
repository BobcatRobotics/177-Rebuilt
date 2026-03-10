package frc.robot.subsystems.Intake;

import org.bobcatrobotics.Util.Tunables.TunableDouble;

import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeState.IntakeGoal;

public class IntakeState {
  public static class IntakeGoal {
    public double position;
    public double speed;
  }

  public enum State {
    IDLE,
    TARGETING
  }

  private State currentState = State.IDLE;
  private IntakeGoal currentSetpoints = new IntakeGoal();


  public IntakeState(){
  }

  /** Set the intake to a predefined state */
  public void setState(State state) {
    this.currentState = state;
  }

  /** Returns the intake outputs based on the current state */
  public void update() {
    switch (currentState) {
      case IDLE -> {
        currentSetpoints.position = Constants.IntakeConstants.idleIntakePosition;
        currentSetpoints.speed = Constants.IntakeConstants.idleRollerSpeed;
      }
      case TARGETING -> {
        currentSetpoints.position = Constants.IntakeConstants.targetIntakePosition;
        currentSetpoints.speed = Constants.IntakeConstants.targetIntakeSpeed;
      }
    }
  }

  public void setCurrentSetPoints(IntakeGoal goal) {
    currentSetpoints = goal;
  }

  public State getCurrentState() {
    return currentState;
  }

  public double getSpeed() {
    return currentSetpoints.speed;
  }

  public double getPosition() {
    return currentSetpoints.position;
  }
}
