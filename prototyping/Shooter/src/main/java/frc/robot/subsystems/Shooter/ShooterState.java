package frc.robot.subsystems.Shooter;

import org.bobcatrobotics.Util.Tunables.TunableDouble;

import frc.robot.Constants;

public class ShooterState {

  /** Output goal for the shooter subsystem */
  public static class ShooterGoal {
    public double flywheelSpeed;
    public double intakeSpeed;
    public double backspinSpeedLeft;
    public double backspinSpeedRight;
  }

  public enum State {
    IDLE,
    MANUAL,
    TARGETING
  }

  private State currentState = State.IDLE;


  // Manual control values
  private TunableDouble manualFlywheelSpeed;
  private TunableDouble manualIntakeSpeed;
  private TunableDouble manualBackspinSpeedLeft;
  private TunableDouble manualBackspinSpeedRight;


  public ShooterState() {
      manualFlywheelSpeed = new TunableDouble("/Shooter/Flywheel/manualFlywheelSetPoint", 0.0);
      manualIntakeSpeed = new TunableDouble("/Shooter/Intake/manualIntakeSetPoint", 0.0);
      manualBackspinSpeedLeft = new TunableDouble("/Shooter/Backspin/manualBackspinSetPointLeft", 0.0);
      manualBackspinSpeedRight = new TunableDouble("/Shooter/Backspin/manualBackspinSetPointRight", 0.0);

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
      double intakeSpeed,
      double backspinSpeedLeft,
      double backspinSpeedRight) {
      manualFlywheelSpeed = new TunableDouble("/Shooter/Flywheel/manualFlywheelSetPoint",
          flywheelSpeed);
      manualIntakeSpeed = new TunableDouble("/Shooter/Intake/manualIntakeSetPoint", intakeSpeed);
      manualBackspinSpeedLeft = new TunableDouble("/Shooter/Backspin/manualBackspinSetPointLeft",
          backspinSpeedLeft);
      manualBackspinSpeedRight = new TunableDouble("/Shooter/Backspin/manualBackspinSetPointRight",
          backspinSpeedLeft);
    currentState = State.MANUAL;
  }

  /** Returns the shooter outputs based on the current state */
  public ShooterGoal getOutput() {
    ShooterGoal goal = new ShooterGoal();

    switch (currentState) {
      case IDLE -> {
        goal.flywheelSpeed = Constants.ShooterConstants.idleFlywheelSpeedRPS;
        goal.intakeSpeed = Constants.ShooterConstants.idleIntakeSpeedRPS;
        goal.backspinSpeedLeft = Constants.ShooterConstants.idleBackspinSpeedLeftRPS;
        goal.backspinSpeedRight = Constants.ShooterConstants.idleBackspinSpeedRightRPS;
      }
      case MANUAL -> {
        goal.flywheelSpeed = manualFlywheelSpeed.get();
        goal.intakeSpeed = manualIntakeSpeed.get();
        goal.backspinSpeedLeft = manualBackspinSpeedLeft.get();
        goal.backspinSpeedRight = manualBackspinSpeedRight.get();
      }

      case TARGETING -> {
        // Placeholder – typically filled in by vision / interpolation
        goal.flywheelSpeed = Constants.ShooterConstants.targetFlywheelSpeedRPS;
        goal.backspinSpeedLeft = Constants.ShooterConstants.targetBackspinSpeedRPS;
        goal.backspinSpeedRight = Constants.ShooterConstants.targetBackspinSpeedRPS;
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

  public double getBackspinSpeedOfLeft() {
    return getOutput().backspinSpeedLeft;
  }
  public double getBackspinSpeedOfRight() {
    return getOutput().backspinSpeedRight;
  }
}
