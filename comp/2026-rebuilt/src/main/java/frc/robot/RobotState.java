package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotState {
    public Alliance alliance;
    private static RobotState instance;
    public Pose2d robotPose = new Pose2d();
    public double vx,vy = 0.0;
    public boolean isRobotAlignedToHub;

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }
}
