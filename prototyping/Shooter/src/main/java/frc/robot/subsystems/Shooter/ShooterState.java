package frc.robot.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

import org.bobcatrobotics.Util.Tunables.TunableDouble;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Modules.ModuleType;

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
  private TunableDouble manualFlywheelSpeed;
  private TunableDouble manualIntakeSpeed;
  private TunableDouble manualBackspinSpeed;

  private List<ModuleType> moduleTypes;

  public ShooterState(String name, List<ModuleType> moduleTypes) {
    this.moduleTypes = moduleTypes;
    this.name = name;
    if (moduleTypes.contains(ModuleType.FLYWHEEL)) {
      manualFlywheelSpeed = new TunableDouble("/Shooter/" + name + "/Flywheel/manualFlywheelSpeedTarget", 0.0);
    }
    if (moduleTypes.contains(ModuleType.INTAKE)) {
      manualIntakeSpeed = new TunableDouble("/Shooter/" + name + "/Intake/manualIntakeSpeedTarget", 0.0);
    }
    if (moduleTypes.contains(ModuleType.BACKSPIN)) {
      manualBackspinSpeed = new TunableDouble("/Shooter/" + name + "/Backspin/manualBackspinSpeedTarget", 0.0);
    }
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
    if (moduleTypes.contains(ModuleType.FLYWHEEL)) {
      manualFlywheelSpeed = new TunableDouble("/Shooter/" + name + "/Flywheel/manualFlywheelSpeedTarget",
          flywheelSpeed);
    }
    if (moduleTypes.contains(ModuleType.INTAKE)) {
      manualIntakeSpeed = new TunableDouble("/Shooter/" + name + "/Intake/manualIntakeSpeedTarget", intakeSpeed);
    }
    if (moduleTypes.contains(ModuleType.BACKSPIN)) {
      manualBackspinSpeed = new TunableDouble("/Shooter/" + name + "/Backspin/manualBackspinSpeedTarget",
          backspinSpeed);
    }
    currentState = State.MANUAL;
  }

  /** Returns the shooter outputs based on the current state */
  public ShooterGoal getOutput() {
    ShooterGoal goal = new ShooterGoal();

    switch (currentState) {
      case IDLE -> {
        goal.flywheelSpeed = Constants.ShooterConstants.idleFlywheelSpeedRPM;
        goal.intakeSpeed = Constants.ShooterConstants.idleIntakeSpeedRPM;
        goal.backspinSpeed = Constants.ShooterConstants.idleBackspinSpeedRPM;
      }
      case MANUAL -> {
        goal.flywheelSpeed = manualFlywheelSpeed.get();
        goal.intakeSpeed = manualIntakeSpeed.get();
        goal.backspinSpeed = manualBackspinSpeed.get();
      }

      case TARGETING -> {
        // Placeholder – typically filled in by vision / interpolation
        goal.flywheelSpeed = Constants.ShooterConstants.targetFlywheelSpeedRPM;
        goal.intakeSpeed = Constants.ShooterConstants.targetIntakeSpeedRPM;
        goal.backspinSpeed = Constants.ShooterConstants.targetBackspinSpeedRPM;
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
