package frc.robot.subsystems.Shooter;

import org.bobcatrobotics.Util.Interpolators.TripleOutputInterpolator;
import org.bobcatrobotics.Util.Tunables.TunableDouble;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotState;

public class ShooterState {

  /** Output goal for the shooter subsystem */
  public static class ShooterGoal {
    public double flywheelSpeed;
    public double hoodSpeed;
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
  public void setManualSpeeds(
      double flywheelSpeed,
      double hoodSpeed) {
    currentState = State.MANUAL;
  }

  /** Returns the shooter outputs based on the current state */
  public void update() {

    switch (currentState) {
      case IDLE -> {
        currentSetpoints.flywheelSpeed = Constants.ShooterConstants.idleFlywheelSpeedRPS;
        currentSetpoints.hoodSpeed = Constants.ShooterConstants.idleHoodSpeedRPS;
      }
      case
          INTERPOLATING -> {
        // Placeholder – typically filled in by vision / interpolation
        double hubDistance = RobotState.getInstance().hubDistance;
        currentSetpoints.flywheelSpeed = RobotState.getInstance().interpolator.getAsList(hubDistance).get(2);
        currentSetpoints.hoodSpeed = RobotState.getInstance().interpolator.getAsList(hubDistance).get(1);
      }
      case TARGETING -> {
        // Placeholder – typically filled in by vision / interpolation
        currentSetpoints.flywheelSpeed = Constants.ShooterConstants.targetFlywheelSpeedRPS;
        currentSetpoints.hoodSpeed = Constants.ShooterConstants.targetHoodSpeedRPS;
      }
    }
  }

  public void setCurrentSetPoints(ShooterGoal goal) {
    currentSetpoints = goal;
  }

  public State getCurrentState() {
    return currentState;
  }

  public double getFlywheelSpeed() {
    return currentSetpoints.flywheelSpeed;
  }

  public double getHoodSpeed() {
    return currentSetpoints.hoodSpeed;
  }
}
