package frc.robot;

import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;

public class RobotState {
  public CharacterizationType characterizationType = CharacterizationType.DRIVE;
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null)
    {
      instance = new RobotState();
    }
    return instance;
  }

  // Save Odometry,
  // Save Swerve Module Details
  // Save Vision Tags

  // Save Shooter State
  private ShooterState.State desiredShooterStateType = ShooterState.State.IDLE;
  private ShooterState desiredShooterState = new ShooterState();

  public ShooterState.State getDesiredShooterStateType() {
    return desiredShooterStateType;
  }

  public ShooterState getShooterState() {
    return desiredShooterState;
  }

  // Save Hopper State
  private HopperState.State desiredHopperStateType = HopperState.State.IDLE;
  private HopperState desiredHopperState  = new HopperState();

  public HopperState.State getDesiredHopperStateType() {
    return desiredHopperStateType;
  }

  public HopperState getHopperState() {
    return desiredHopperState;
  }

  // Save Intake State
  private IntakeState.State desiredIntakeStateType = IntakeState.State.IDLE;
  private IntakeState desiredIntakeState = new IntakeState();

  public IntakeState.State getDesiredIntakeStateType() {
    return desiredIntakeStateType;
  }

  public IntakeState getIntakeState() {
    return desiredIntakeState;
  }
}
