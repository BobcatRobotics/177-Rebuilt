package frc.vision.cal;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Conversions between Limelight conventions and WPILib geometry.
 *
 * <p>Limelight robot space (what {@code VisionConstants.cameraConstants} and
 * {@code setCameraPose_RobotSpace} use): forward, side (right-positive), up in meters; roll, pitch
 * (positive = tilted up), yaw (positive = turned left / counter-clockwise) in degrees.
 *
 * <p>WPILib: x forward, y left, z up; positive pitch tips the camera's view down.
 */
public final class CamGeom {
  private CamGeom() {}

  /** Robot-to-camera transform in WPILib convention from Limelight mount values. */
  public static Transform3d mountFromLimelight(
      double forward, double side, double up, double rollDeg, double pitchDeg, double yawDeg) {
    return new Transform3d(
        new Translation3d(forward, -side, up),
        new Rotation3d(
            Math.toRadians(rollDeg), Math.toRadians(-pitchDeg), Math.toRadians(yawDeg)));
  }

  /** Inverse of {@link #mountFromLimelight}: {forward, side, up, roll, pitch, yaw}. */
  public static double[] mountToLimelight(Transform3d mount) {
    Rotation3d r = mount.getRotation();
    return new double[] {
      mount.getX(),
      -mount.getY(),
      mount.getZ(),
      Math.toDegrees(r.getX()),
      -Math.toDegrees(r.getY()),
      Math.toDegrees(r.getZ())
    };
  }

  /**
   * Tag center position in the WPILib camera frame (x out of the lens, y left, z up) from a
   * Limelight detection: horizontal/vertical angles from the principal point (txnc right-positive,
   * tync up-positive, degrees) and straight-line distance to the tag center (meters).
   */
  public static Translation3d tagInCamera(double txncDeg, double tyncDeg, double distanceM) {
    double x = 1.0;
    double y = -Math.tan(Math.toRadians(txncDeg));
    double z = Math.tan(Math.toRadians(tyncDeg));
    double n = Math.sqrt(x * x + y * y + z * z);
    return new Translation3d(x / n * distanceM, y / n * distanceM, z / n * distanceM);
  }

  /** Inverse of {@link #tagInCamera}: {txncDeg, tyncDeg, distanceM}. Used by tests. */
  public static double[] detectionFromCamera(Translation3d tagInCam) {
    double tx = -Math.toDegrees(Math.atan2(tagInCam.getY(), tagInCam.getX()));
    double ty = Math.toDegrees(Math.atan2(tagInCam.getZ(), tagInCam.getX()));
    return new double[] {tx, ty, tagInCam.getNorm()};
  }

  /** Field position of a tag seen from a camera, given the robot pose and camera mount. */
  public static Translation3d tagInField(
      Pose3d robot, Transform3d mount, Translation3d tagInCam) {
    Pose3d cam = robot.transformBy(mount);
    return cam.transformBy(new Transform3d(tagInCam, Rotation3d.kZero)).getTranslation();
  }
}
