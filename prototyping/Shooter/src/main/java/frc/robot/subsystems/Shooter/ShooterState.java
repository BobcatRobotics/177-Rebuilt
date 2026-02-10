package frc.robot.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

import org.bobcatrobotics.Util.Tunables.TunableDouble;

import frc.robot.Constants;

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

  private String name;

  // Manual control values
  private TunableDouble manualFlywheelSpeed ;
  private TunableDouble manualIntakeSpeed;
  private TunableDouble manualBackspinSpeed;

  public ShooterState(String name) {
    this.name = name;
    manualFlywheelSpeed = new TunableDouble("/Shooter/" + name + "/Flywheel/manualFlywheelSpeedTarget", 0.0);
    manualIntakeSpeed = new TunableDouble("/Shooter/" + name + "/Intake/manualIntakeSpeedTarget", 0.0);
    manualBackspinSpeed = new TunableDouble("/Shooter/" + name + "/Backspin/manualBackspinSpeedTarget", 0.0);
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
      double backspinSpeed) {
    manualFlywheelSpeed = new TunableDouble("/Shooter/" + name + "/Flywheel/manualFlywheelSpeedTarget", flywheelSpeed);
    manualIntakeSpeed = new TunableDouble("/Shooter/" + name + "/Intake/manualIntakeSpeedTarget", intakeSpeed);
    manualBackspinSpeed = new TunableDouble("/Shooter/" + name + "/Backspin/manualBackspinSpeedTarget", backspinSpeed);
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
