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
    MANUAL,
    TARGETING
  }

  private State currentState = State.IDLE;
  private IntakeGoal currentSetpoints = new IntakeGoal();

    // Manual control values
  private TunableDouble manualIntakePosition;
  private TunableDouble manualIntakeSpeed;

  public IntakeState(){
    manualIntakePosition = new TunableDouble("/Intake/manualPosition",0.0);
    manualIntakeSpeed = new TunableDouble("/Intake/manualSpeed",0.0);
  }

  /** Set the intake to a predefined state */
  public void setState(State state) {
    this.currentState = state;
  }

    /**
   * Set all intake speed and position at once and switch to MANUAL mode
   */
    public void setManualSpeedAndPosition(
        double intakeSpeed,
        double intakePosition) {
      manualIntakePosition = new TunableDouble("/Intake/manualPosition",
          intakePosition);
      manualIntakeSpeed = new TunableDouble("/Intake/manualSpeed",
          intakeSpeed);
      currentState = State.MANUAL;
    }

  /** Returns the intake outputs based on the current state */
  public void update() {
    switch (currentState) {
      case IDLE -> {
        currentSetpoints.position = Constants.IntakeConstants.idleIntakePosition;
        currentSetpoints.speed = Constants.IntakeConstants.idleRollerSpeed;
      }
      case MANUAL -> {
        currentSetpoints.position = manualIntakePosition.get();
        currentSetpoints.speed = manualIntakeSpeed.get();
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
