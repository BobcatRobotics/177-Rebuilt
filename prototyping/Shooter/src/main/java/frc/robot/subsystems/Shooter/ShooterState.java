package frc.robot.subsystems.Shooter;

import frc.robot.Constants;
import org.bobcatrobotics.Util.Tunables.TunableDouble;

public class ShooterState {

  /** Output goal for the shooter subsystem */
  public static class ShooterGoal {
    public double flywheelSpeed;
    public double intakeSpeed;
    public double backspinSpeed;
  }

  public enum State {
    IDLE,
    MANUAL,
    TARGETING
  }

  private State currentState = State.IDLE;

  // Manual control values
  private TunableDouble manualFlywheelSpeed = new TunableDouble("Shooter/manualFlywheelSpeed", 0.0);
  private TunableDouble manualIntakeSpeed = new TunableDouble("Shooter/manualIntakeSpeed", 0.0);
  private TunableDouble manualBackspinSpeed = new TunableDouble("Shooter/manualBackspinSpeed", 0.0);

  /** Set the shooter to a predefined state */
  public void setState(State state) {
    this.currentState = state;
  }

  /**
   * Set all shooter speeds at once and switch to MANUAL mode
   */
  public void setManualSpeeds(
      double flywheelSpeed,
      double intakeSpeed,
      double backspinSpeed) {
    manualFlywheelSpeed = new TunableDouble("Shooter/manualFlywheelSpeed", flywheelSpeed);
    manualIntakeSpeed = new TunableDouble("Shooter/manualIntakeSpeed", intakeSpeed);
    manualBackspinSpeed = new TunableDouble("Shooter/manualBackspinSpeed", backspinSpeed);

    currentState = State.MANUAL;
  }

  /** Returns the shooter outputs based on the current state */
  public ShooterGoal getOutput() {
    ShooterGoal goal = new ShooterGoal();

    switch (currentState) {
      case IDLE -> {
        goal.flywheelSpeed = Constants.ShooterConstants.idleFlywheelSpeedRPS;
        goal.intakeSpeed = Constants.ShooterConstants.idleIntakeSpeedRPS;
        goal.backspinSpeed = Constants.ShooterConstants.idleBackspinSpeedRPS;
      }

      case MANUAL -> {
        goal.flywheelSpeed = manualFlywheelSpeed.get();
        goal.intakeSpeed = manualIntakeSpeed.get();
        goal.backspinSpeed = manualBackspinSpeed.get();
      }

      case TARGETING -> {
        // Placeholder – typically filled in by vision / interpolation
        goal.flywheelSpeed = Constants.ShooterConstants.targetFlywheelSpeedRPS;
        goal.intakeSpeed = Constants.ShooterConstants.targetIntakeSpeedRPS;
        goal.backspinSpeed = Constants.ShooterConstants.targetBackspinSpeedRPS;
      }
    }

    return goal;
  }

  public State getCurrentState() {
    return currentState;
  }

  public double getFlywheelSpeed() {
    return getOutput().flywheelSpeed;
  }

  public double getIntakeSpeed() {
    return getOutput().intakeSpeed;
  }

  public double getBackspinSpeed() {
    return getOutput().backspinSpeed;
  }
}
