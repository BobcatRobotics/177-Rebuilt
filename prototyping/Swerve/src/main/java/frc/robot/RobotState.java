package frc.robot;

public class RobotState {
    private static RobotState instance;
      public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // Save Odometry,
  // Save Swerve Module Details
  // Save Vision Tags

}
