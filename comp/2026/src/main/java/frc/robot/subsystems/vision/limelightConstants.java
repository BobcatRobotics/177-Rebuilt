package frc.robot.subsystems.vision;


public class limelightConstants {
  public final String name;
  public final double forward;
  public final double side;
  public final double up;
  public final double roll;
  public final double pitch;
  public final double yaw;

  /**
   * Limelight Constants for offsets
   *
   * @param name
   * @param forward
   * @param side
   * @param up
   * @param roll
   * @param pitch
   * @param yaw
   */
  public limelightConstants(
      String name,
      double forward,
      double side,
      double up,
      double roll,
      double pitch,
      double yaw) {
    this.name = name;
    this.forward = forward; // degrees obviously
    this.side = side;
    this.up = up;
    this.roll = roll;
    this.pitch = pitch;
    this.yaw = yaw;
    // upload offsets to limelight
    LimelightHelpers.setCameraPose_RobotSpace(name, forward, side, up, roll, pitch, yaw);
  }
}

// enum LLTYPE {
//   LL3,
//   LL3G,
//   LL4
// }
