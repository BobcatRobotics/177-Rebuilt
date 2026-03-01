package frc.robot;

import frc.robot.subystems.Climber.ClimberState;

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

 

  // Save Climber State
  private ClimberState.State desiredClimberStateType = ClimberState.State.MANUAL;
  private ClimberState desiredClimberState;

  public ClimberState.State getDesiredIntakeStateType() {
    return desiredClimberStateType;
  }

  public ClimberState getClimberState() {
    return desiredClimberState;
  }
}
