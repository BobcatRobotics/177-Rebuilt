package frc.robot;

import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;

public class RobotState {
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
  private ShooterState.State desiredShooterStateType = ShooterState.State.MANUAL;
  private ShooterState desiredShooterState;

  public ShooterState.State getDesiredShooterStateType() {
    return desiredShooterStateType;
  }

  public ShooterState getShooterState() {
    return desiredShooterState;
  }

  // Save Hopper State
  private HopperState.State desiredHopperStateType = HopperState.State.MANUAL;
  private HopperState desiredHopperState;

  public HopperState.State getDesiredHopperStateType() {
    return desiredHopperStateType;
  }

  public HopperState getHopperState() {
    return desiredHopperState;
  }

  // Save Intake State
  private IntakeState.State desiredIntakeStateType = IntakeState.State.MANUAL;
  private IntakeState desiredIntakeState;

  public IntakeState.State getDesiredIntakeStateType() {
    return desiredIntakeStateType;
  }

  public IntakeState getIntakeState() {
    return desiredIntakeState;
  }
}
