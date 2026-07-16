package frc.robot.subsystems.Carwash;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class CarwashState {

  /** Output goal for the shooter subsystem */
  public static class CarwashGoal {
    public double intakeSpeed;
  }

  public enum State {
    IDLE,
    MANUAL,
    INTERPOLATING,
    TARGETING
  }

  private State currentState = State.IDLE;
  private CarwashGoal currentSetpoints = new CarwashGoal();

  // Manual control values


  public CarwashState() {

  }

  /** Set the shooter to a predefined state */
  public void setState(State state) {
    this.currentState = state;
  }

  /**
   * Set all shooter speeds at once and switch to MANUAL mode
   */
  public void setManualSpeeds(
      double intakeSpeed) {
    currentState = State.MANUAL;
  }

  /** Returns the shooter outputs based on the current state */
  public void update() {

    switch (currentState) {
      case IDLE -> {
        currentSetpoints.intakeSpeed = Constants.CarwashConstants.idleIntakeSpeedRPS;
      }
      case INTERPOLATING -> {
        // Placeholder – typically filled in by vision / interpolation
        double hubDistance = frc.robot.RobotState.getInstance().hubDistance;
        currentSetpoints.intakeSpeed = 80;
      }
      case TARGETING -> {
        // Placeholder – typically filled in by vision / interpolation
        currentSetpoints.intakeSpeed = Constants.CarwashConstants.targetIntakeSpeedRPS;
      }
    }
    Logger.recordOutput("Carwash/GoalSpeed", currentSetpoints.intakeSpeed);
  }

  public void setCurrentSetPoints(CarwashGoal goal){
    currentSetpoints = goal;
  }

  public State getCurrentState() {
    return currentState;
  }

  public double getIntakeSpeed() {
    return currentSetpoints.intakeSpeed;
  }
}
