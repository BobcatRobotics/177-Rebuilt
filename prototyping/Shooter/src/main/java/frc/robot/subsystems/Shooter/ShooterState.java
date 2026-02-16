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
  private ShooterGoal currentSetpoints = new ShooterGoal();

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
      manualFlywheelSpeed = new TunableDouble("/Shooter/Flywheel/manualFlywheelSetPoint",flywheelSpeed);
      manualIntakeSpeed = new TunableDouble("/Shooter/Intake/manualIntakeSetPoint", intakeSpeed);
      manualBackspinSpeedLeft = new TunableDouble("/Shooter/Backspin/manualBackspinSetPointLeft",backspinSpeedLeft);
      manualBackspinSpeedRight = new TunableDouble("/Shooter/Backspin/manualBackspinSetPointRight",backspinSpeedRight);
    currentState = State.MANUAL;
  }

  /** Returns the shooter outputs based on the current state */
  public void update() {

    switch (currentState) {
      case IDLE -> {
        currentSetpoints.flywheelSpeed = Constants.ShooterConstants.idleFlywheelSpeedRPS;
        currentSetpoints.intakeSpeed = Constants.ShooterConstants.idleIntakeSpeedRPS;
        currentSetpoints.backspinSpeedLeft = Constants.ShooterConstants.idleBackspinSpeedLeftRPS;
        currentSetpoints.backspinSpeedRight = Constants.ShooterConstants.idleBackspinSpeedRightRPS;
      }
      case MANUAL -> {
        currentSetpoints.flywheelSpeed = manualFlywheelSpeed.get();
        currentSetpoints.intakeSpeed = manualIntakeSpeed.get();
        currentSetpoints.backspinSpeedLeft = manualBackspinSpeedLeft.get();
        currentSetpoints.backspinSpeedRight = manualBackspinSpeedRight.get();
      }

      case TARGETING -> {
        // Placeholder – typically filled in by vision / interpolation
        currentSetpoints.flywheelSpeed = Constants.ShooterConstants.targetFlywheelSpeedRPS;
        currentSetpoints.intakeSpeed = Constants.ShooterConstants.targetIntakeSpeedRPS;
        currentSetpoints.backspinSpeedLeft = Constants.ShooterConstants.targetBackspinSpeedLeftRPS;
        currentSetpoints.backspinSpeedRight = Constants.ShooterConstants.targetBackspinSpeedRightRPS;
      }
    }
  }

  public void setCurrentSetPoints(ShooterGoal goal){
    currentSetpoints = goal;
  }

  public State getCurrentState() {
    return currentState;
  }

  public double getFlywheelSpeed() {
    return currentSetpoints.flywheelSpeed;
  }

  public double getIntakeSpeed() {
    return currentSetpoints.intakeSpeed;
  }

  public double getBackspinSpeedOfLeft() {
    return currentSetpoints.backspinSpeedLeft;
  }
  public double getBackspinSpeedOfRight() {
    return currentSetpoints.backspinSpeedRight;
  }
}
