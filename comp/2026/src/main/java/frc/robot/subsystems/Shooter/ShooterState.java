package frc.robot.subsystems.Shooter;

import org.bobcatrobotics.Util.Interpolators.TripleOutputInterpolator;
import org.bobcatrobotics.Util.Tunables.TunableDouble;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotState;

public class ShooterState {

  /** Output goal for the shooter subsystem */
  public static class ShooterGoal {
    public double leftDumperSpeed;
    public double rightDumperSpeed;
    public double hoodPosition;
  }

  public enum State {
    IDLE,
    MANUAL,
    INTERPOLATING,
    TARGETING
  }

  private State currentState = State.IDLE;
  private ShooterGoal currentSetpoints = new ShooterGoal();


  // Manual control values

  public ShooterState() {

  }

  /** Set the shooter to a predefined state */
  public void setState(State state) {
    this.currentState = state;
  }

  /**
   * Set all shooter speeds at once and switch to MANUAL mode
   */
  public void setManualShot(
      double leftDumperSpeed,
      double rightDumperSpeed,
      double hoodPosition) {
    currentState = State.MANUAL;
  }

  /** Returns the shooter outputs based on the current state */
  public void update() {

    switch (currentState) {
      case IDLE -> {
        currentSetpoints.leftDumperSpeed = Constants.ShooterConstants.idleDumperSpeed;
        currentSetpoints.rightDumperSpeed = Constants.ShooterConstants.idleDumperSpeed;
        currentSetpoints.hoodPosition = Constants.ShooterConstants.idleHoodPosition;
      }
      case
          INTERPOLATING -> {
        // Placeholder – typically filled in by vision / interpolation
        double hubDistance = RobotState.getInstance().hubDistance;
        currentSetpoints.leftDumperSpeed = RobotState.getInstance().interpolator.getAsList(hubDistance).get(1);
        currentSetpoints.rightDumperSpeed = RobotState.getInstance().interpolator.getAsList(hubDistance).get(1);
        // currentSetpoints.hoodSpeed = RobotState.getInstance().interpolator.getAsList(hubDistance).get(1);
        currentSetpoints.hoodPosition = RobotState.getInstance().interpolator.getAsList(hubDistance).get(2);;

      }
      case TARGETING -> {
        // Placeholder – typically filled in by vision / interpolation
        currentSetpoints.leftDumperSpeed = Constants.ShooterConstants.targetDumperSpeed;
        currentSetpoints.rightDumperSpeed = Constants.ShooterConstants.targetDumperSpeed;
        currentSetpoints.hoodPosition = Constants.ShooterConstants.targetHoodPosition;
      }
    }        
    Logger.recordOutput("Shooter/rightDumper/GoalSpeeds", currentSetpoints.leftDumperSpeed);
    Logger.recordOutput("Shooter/leftDumper/GoalSpeeds", currentSetpoints.rightDumperSpeed);
    Logger.recordOutput("Shooter/adjustableHood/GoalPosition", currentSetpoints.hoodPosition);
  }

  public void setCurrentSetPoints(ShooterGoal goal) {
    currentSetpoints = goal;
  }

  public State getCurrentState() {
    return currentState;
  }

  public double getLeftDumperSpeed() {
    return currentSetpoints.leftDumperSpeed;
  }

  public double getRightDumperSpeed() {
    return currentSetpoints.rightDumperSpeed;
  }

  public double getAdjustableHoodPosition(){
    return currentSetpoints.hoodPosition;
  }
}
