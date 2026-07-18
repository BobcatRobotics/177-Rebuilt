package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotInfo {
    public Alliance alliance;
    private static RobotInfo instance;
    public Pose2d robotPose = new Pose2d();
    public Pose2d futurePose = new Pose2d();
    public double vx,vy = 0.0;
    public boolean isRobotAlignedToHub;
    public double hubDistance = 0.0;
    public static RobotInfo getInstance() {
        if (instance == null) {
            instance = new RobotInfo();
        }
        return instance;
    }    
}
