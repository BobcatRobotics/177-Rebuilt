package frc.robot.subsystems.Shooter;

import org.bobcatrobotics.GameSpecific.Rebuilt.HubData;
import org.bobcatrobotics.Util.Interpolators.TripleOutputInterpolator;
import org.bobcatrobotics.Util.Tunables.TunableDouble;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotState;

public class ShooterState {
  private LinearFilter clampedDistance;

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
    INTERPOLATEDPASSING,
    TARGETING
  }

  private State currentState = State.IDLE;
  private ShooterGoal currentSetpoints = new ShooterGoal();

  // 2910-style shot latch: while a shoot command keeps requesting it (every loop while feeding),
  // update() reuses the distances from the loop before the lock started, so pose noise can't move
  // the drum speed or hood mid-volley. The request expires one loop after the last call.
  private boolean distanceLockRequested = false;
  private boolean distanceLocked = false;
  private double lockedHubDistance;
  private double lockedClampedHubDistance;
  private double lastHubDistance;
  private double lastClampedHubDistance;

  // Manual control values

  public ShooterState() {
clampedDistance = LinearFilter.movingAverage(5);
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
    if (distanceLockRequested && !distanceLocked) {
      lockedHubDistance = lastHubDistance;
      lockedClampedHubDistance = lastClampedHubDistance;
    }
    distanceLocked = distanceLockRequested;
    distanceLockRequested = false;

    double hubDistance = distanceLocked ? lockedHubDistance : RobotState.getInstance().hubDistance;
    double clampedHubDistance = hubDistance;
    switch (currentState) {
      case IDLE -> {
        currentSetpoints.leftDumperSpeed = Constants.ShooterConstants.idleDumperSpeed;
        currentSetpoints.rightDumperSpeed = Constants.ShooterConstants.idleDumperSpeed;
        currentSetpoints.hoodPosition = Constants.ShooterConstants.idleHoodPosition;
      }
      case
          INTERPOLATING -> {
        // Placeholder – typically filled in by vision / interpolation
        clampedHubDistance = distanceLocked ? lockedClampedHubDistance : clampedDistance.calculate(hubDistance);
        double dumperSpeed = RobotState.getInstance().interpolator.getAsList(hubDistance).get(1);
        currentSetpoints.leftDumperSpeed = dumperSpeed;
        currentSetpoints.rightDumperSpeed = dumperSpeed;
        currentSetpoints.hoodPosition = RobotState.getInstance().interpolator.getAsList(clampedHubDistance).get(2);
        ;

      }
      case
          INTERPOLATEDPASSING -> {
        // Placeholder – typically filled in by vision / interpolation
        clampedHubDistance = distanceLocked ? lockedClampedHubDistance : clampedDistance.calculate(hubDistance);
        double dumperSpeed = RobotState.getInstance().passingInterpolator.getAsList(hubDistance).get(1);
        currentSetpoints.leftDumperSpeed = dumperSpeed;
        currentSetpoints.rightDumperSpeed = dumperSpeed;
        currentSetpoints.hoodPosition = RobotState.getInstance().passingInterpolator.getAsList(clampedHubDistance).get(2);
        ;

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
    // Live distance from the pose; ShotHubDistance is what the shot used (differs while locked)
    Logger.recordOutput("Shooter/hubDistance", RobotState.getInstance().hubDistance);
    Logger.recordOutput("Shooter/ShotHubDistance", hubDistance);
    Logger.recordOutput("Shooter/clamedHubDistance", clampedHubDistance);
    Logger.recordOutput("Shooter/DistanceLocked", distanceLocked);
    lastHubDistance = hubDistance;
    lastClampedHubDistance = clampedHubDistance;
  }

  /**
   * Freeze the shot distance (drum speed and hood) at its current value. Call every loop while
   * feeding; the lock releases on the first loop it isn't requested.
   */
  public void requestDistanceLock() {
    distanceLockRequested = true;
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

  public double getAdjustableHoodPosition() {
    return currentSetpoints.hoodPosition;
  }
}
