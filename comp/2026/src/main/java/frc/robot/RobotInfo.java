package frc.robot;

import java.util.HashMap;
import java.util.List;

import org.bobcatrobotics.Util.CANDeviceDetails;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotInfo {
  public CharacterizationType characterizationType = CharacterizationType.DRIVE;
  public Alliance alliance;
  private static RobotInfo instance;
  public double vx = 0.0;
  public double vy = 0.0;

  public static RobotInfo getInstance() {
    if (instance == null)
    {
      instance = new RobotInfo();
    }
    return instance;
  }

  // Save Odometry,
  // Save Swerve Module Details
  // Save Vision Tags
 public Pose2d robotPose = new Pose2d();
  public boolean shooterUpToSpeed = false;
  public boolean hubInrange = false;
  public double hubDistance = 0.0;
  // used by new align command not to be confused with hubInRange;
  public boolean isRobotAlignedToHub = false;

  public double depoDistance = 0.0;
  public double outpostDistance = 0.0;
  public double targettedDistance = 0.0;
  public boolean isRobotAlignedToPassingLoc = false;

  public HashMap<String, List<CANDeviceDetails>> devices = new HashMap<String, List<CANDeviceDetails>>();
}
