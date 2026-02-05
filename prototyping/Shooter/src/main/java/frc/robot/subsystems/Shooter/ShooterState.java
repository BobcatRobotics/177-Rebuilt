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
    MANUALBOTH,
    MANUALLEFT,
    MANUALRIGHT,
    TARGETINGBOTH,
    TARGETINGLEFT,
    TARGETINGRIGHT
  }

  private State currentState = State.IDLE;

  // Manual control values
  private TunableDouble manualFlywheelSpeedRight = new TunableDouble("/Shooter/MainMotor/Right/manualFlywheelSpeedTarget", 0.0);
  private TunableDouble manualIntakeSpeedRight = new TunableDouble("/Shooter/Intake/Right/manualIntakeSpeedTarget", 0.0);
  private TunableDouble manualBackspinSpeedRight = new TunableDouble("/Shooter/Backspin/Right/manualBackspinSpeedTarget", 0.0);
  private TunableDouble manualFlywheelSpeedLeft = new TunableDouble("/Shooter/MainMotor/Left/manualFlywheelSpeedTarget", 0.0);
  private TunableDouble manualIntakeSpeedLeft = new TunableDouble("/Shooter/Intake/Left/manualIntakeSpeedTarget", 0.0);
  private TunableDouble manualBackspinSpeedLeft = new TunableDouble("/Shooter/Backspin/Left/manualBackspinSpeedTarget", 0.0);

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
    manualFlywheelSpeedRight = new TunableDouble("/Shooter/MainMotor/Right/manualFlywheelSpeedTarget", flywheelSpeed);
    manualIntakeSpeedRight = new TunableDouble("/Shooter/Intake/Right/manualIntakeSpeedTarget", intakeSpeed);
    manualBackspinSpeedRight = new TunableDouble("/Shooter/Backspin/Right/manualBackspinSpeedTarget", backspinSpeed);
    manualFlywheelSpeedLeft = new TunableDouble("/Shooter/MainMotor/Left/manualFlywheelSpeedTarget", flywheelSpeed);
    manualIntakeSpeedLeft = new TunableDouble("/Shooter/Intake/Left/manualIntakeSpeedTarget", intakeSpeed);
    manualBackspinSpeedLeft = new TunableDouble("/Shooter/Backspin/Left/manualBackspinSpeedTarget", backspinSpeed);

    currentState = State.MANUALBOTH;
  }


  /** Returns the shooter outputs based on the current state */
  public List<ShooterGoal> getOutput() {
    ShooterGoal leftGoal = new ShooterGoal();
    ShooterGoal rightGoal = new ShooterGoal();
    List<ShooterGoal> goals = new ArrayList<ShooterGoal>();

    switch (currentState) {
      case IDLE -> {
        leftGoal.flywheelSpeed = Constants.ShooterConstants.idleFlywheelSpeedRPS;
        leftGoal.intakeSpeed = Constants.ShooterConstants.idleIntakeSpeedRPS;
        leftGoal.backspinSpeed = Constants.ShooterConstants.idleBackspinSpeedRPS;
        rightGoal.flywheelSpeed = Constants.ShooterConstants.idleFlywheelSpeedRPS;
        rightGoal.intakeSpeed = Constants.ShooterConstants.idleIntakeSpeedRPS;
        rightGoal.backspinSpeed = Constants.ShooterConstants.idleBackspinSpeedRPS;
        goals.add(leftGoal);
        goals.add(rightGoal);
      }
      case MANUALLEFT -> {
        leftGoal.flywheelSpeed = manualFlywheelSpeedLeft.get();
        leftGoal.intakeSpeed = manualIntakeSpeedLeft.get();
        leftGoal.backspinSpeed = manualBackspinSpeedLeft.get();
        goals.add(leftGoal);
      }
      case MANUALRIGHT -> {
        rightGoal.flywheelSpeed = manualFlywheelSpeedRight.get();
        rightGoal.intakeSpeed = manualIntakeSpeedRight.get();
        rightGoal.backspinSpeed = manualBackspinSpeedRight.get();
        goals.add(rightGoal);
      }
      case MANUALBOTH -> {
        leftGoal.flywheelSpeed = manualFlywheelSpeedLeft.get();
        leftGoal.intakeSpeed = manualIntakeSpeedLeft.get();
        leftGoal.backspinSpeed = manualBackspinSpeedLeft.get();
        rightGoal.flywheelSpeed = manualFlywheelSpeedRight.get();
        rightGoal.intakeSpeed = manualIntakeSpeedRight.get();
        rightGoal.backspinSpeed = manualBackspinSpeedRight.get();
        goals.add(leftGoal);
        goals.add(rightGoal);
      }
      case TARGETINGLEFT -> {
        // Placeholder – typically filled in by vision / interpolation
        leftGoal.flywheelSpeed = Constants.ShooterConstants.LeftShooterConstants.targetFlywheelSpeedRPS;
        leftGoal.intakeSpeed = Constants.ShooterConstants.LeftShooterConstants.targetFlywheelSpeedRPS;
        leftGoal.backspinSpeed = Constants.ShooterConstants.LeftShooterConstants.targetFlywheelSpeedRPS;
        goals.add(leftGoal);
      }
      case TARGETINGRIGHT -> {
        // Placeholder – typically filled in by vision / interpolation
        rightGoal.flywheelSpeed = Constants.ShooterConstants.RightShooterConstants.targetFlywheelSpeedRPS;
        rightGoal.intakeSpeed = Constants.ShooterConstants.RightShooterConstants.targetFlywheelSpeedRPS;
        rightGoal.backspinSpeed = Constants.ShooterConstants.RightShooterConstants.targetFlywheelSpeedRPS;
        goals.add(rightGoal);
      }
      case TARGETINGBOTH -> {
        // Placeholder – typically filled in by vision / interpolation
        leftGoal.flywheelSpeed = Constants.ShooterConstants.LeftShooterConstants.targetFlywheelSpeedRPS;
        leftGoal.intakeSpeed = Constants.ShooterConstants.LeftShooterConstants.targetFlywheelSpeedRPS;
        leftGoal.backspinSpeed = Constants.ShooterConstants.LeftShooterConstants.targetFlywheelSpeedRPS;
        rightGoal.flywheelSpeed = Constants.ShooterConstants.RightShooterConstants.targetFlywheelSpeedRPS;
        rightGoal.intakeSpeed = Constants.ShooterConstants.RightShooterConstants.targetFlywheelSpeedRPS;
        rightGoal.backspinSpeed = Constants.ShooterConstants.RightShooterConstants.targetFlywheelSpeedRPS;
        goals.add(leftGoal);
        goals.add(rightGoal);
      }
    }

    return goals;
  }

  public State getCurrentState() {
    return currentState;
  }

  public double getFlywheelSpeed(int index) {
    return getOutput().get(index).flywheelSpeed;
  }

  public double getIntakeSpeed(int index) {
    return getOutput().get(index).intakeSpeed;
  }

  public double getBackspinSpeed(int index) {
    return getOutput().get(index).backspinSpeed;
  }
}
