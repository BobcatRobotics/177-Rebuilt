package frc.vision.cal;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

/**
 * Describes a configured mount in words a person can check by looking at the robot, so a camera
 * that's set up backwards or on the wrong side gets caught even if it never sees a tag.
 *
 * <p>Inputs use VisionConstants / Limelight conventions: side right-positive, pitch up-positive,
 * yaw left (counter-clockwise) positive. Field directions use the WPILib blue-origin frame: +x
 * points at the red alliance wall, +y points at the left side wall as seen from the blue alliance
 * station.
 */
public final class MountWords {
  private MountWords() {}

  /**
   * One-time explanation of "front", "left" and "right" for the robot-relative questions. The
   * robot's front is the code's +x; it's named by the camera that faces it (yaw nearest 0), since
   * "front" is ambiguous on a robot with cameras facing both ways.
   */
  public static String frame(List<Cameras.Camera> cams) {
    return "Picture the robot from above, standing behind it, with its FRONT - "
        + frontWords(cams)
        + " - pointing away from you toward the center of the field.\n"
        + "\"Left\" and \"right\" below are YOUR left and right in that view (the robot's own left/right).\n"
        + "\"Forward\"/\"back\" are toward/away from that front.\n"
        + "Measure to the center of the camera lens: forward/back and left/right from the center of the "
        + "robot's frame, height up from the floor. A difference of more than ~1 in is worth a second look.";
  }

  /** Names the robot's front by the camera that looks forward (yaw nearest 0, furthest forward). */
  public static String frontWords(List<Cameras.Camera> cams) {
    Cameras.Camera front = null;
    for (Cameras.Camera c : cams) {
      if (Math.abs(MountCheck.wrap(c.llMount()[5])) < 30
          && (front == null || c.llMount()[0] > front.llMount()[0])) {
        front = c;
      }
    }
    return front != null
        ? "the side " + front.name() + " is on (that camera looks forward)"
        : "the robot's +x side in the code";
  }

  /** e.g. "mounted 0.43 m back and 0.25 m left of center, 0.48 m up; pointing backward; tilted 26° up". */
  public static String robotRelative(double[] ll) {
    String where = String.format(
        "mounted %s and %s of the robot's center, %.2f m (%.1f in) up",
        along(ll[0], "forward", "back"),
        along(-ll[1], "left", "right"),
        ll[2],
        ll[2] / 0.0254);
    String tilt =
        Math.abs(ll[4]) < 2
            ? "level (no tilt)"
            : String.format("tilted %.0f° %s", Math.abs(ll[4]), ll[4] > 0 ? "up" : "down");
    return where + "; lens pointing " + robotDirection(ll[5]) + "; " + tilt;
  }

  /** Direction relative to the robot for a yaw (deg, left-positive). */
  public static String robotDirection(double yawDeg) {
    return eightWay(
        yawDeg,
        new String[] {
          "forward", "forward-left", "left", "backward-left", "backward", "backward-right", "right",
          "forward-right"
        });
  }

  /** One-time explanation of the wall names used by {@link #fieldDirection}. */
  public static final String FIELD_FRAME =
      "Wall names: picture yourself standing at the BLUE alliance driver station looking across the field "
          + "at the red alliance wall. The \"left side wall\" is the long wall on your left, the \"right side "
          + "wall\" is on your right.";

  /** Field direction a camera points when the robot heading is {@code robotHeading}. */
  public static String fieldDirection(Rotation2d robotHeading, double yawDeg) {
    double deg = robotHeading.getDegrees() + yawDeg;
    return eightWay(
        deg,
        new String[] {
          "the red alliance wall",
          "the corner of the red alliance wall and the left side wall",
          "the left side wall",
          "the corner of the blue alliance wall and the left side wall",
          "the blue alliance wall",
          "the corner of the blue alliance wall and the right side wall",
          "the right side wall",
          "the corner of the red alliance wall and the right side wall"
        });
  }

  /** Robot facing, in the same field words. */
  public static String robotFacing(Rotation2d heading) {
    return fieldDirection(heading, 0);
  }

  private static String eightWay(double deg, String[] names) {
    double d = ((deg % 360) + 360) % 360;
    int idx = (int) Math.round(d / 45.0) % 8;
    return names[idx];
  }

  private static String along(double m, String pos, String neg) {
    if (Math.abs(m) < 0.02) {
      return "centered " + pos + "/" + neg;
    }
    return String.format("%.2f m (%.1f in) %s", Math.abs(m), Math.abs(m) / 0.0254, m > 0 ? pos : neg);
  }
}
