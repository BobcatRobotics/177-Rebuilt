package frc.vision.cal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.vision.cal.FieldConfig.RobotSide;

/** Robot pose for a robot parked with one bumper face flush below/in front of a tag. */
public final class SpotMath {
  private SpotMath() {}

  /**
   * @param tag tag pose on the field; its heading points out of the tag face into the field
   * @param touching which bumper face is toward the tag
   * @param faceGapM distance from the tag's plane to that bumper face (0 = flush)
   * @param lateralM sideways shift along the tag face, positive = to the tag's left as seen from
   *     the field
   * @param lengthM bumper-to-bumper length (front-back)
   * @param widthM bumper-to-bumper width (left-right)
   */
  public static Pose2d tagRelative(
      Pose2d tag,
      RobotSide touching,
      double faceGapM,
      double lateralM,
      double lengthM,
      double widthM) {
    Rotation2d normal = tag.getRotation();
    Translation2d out = new Translation2d(normal.getCos(), normal.getSin());
    // "Tag's left as seen from the field" = looking at the tag, so the tag's own right side
    Translation2d side = new Translation2d(normal.getSin(), -normal.getCos());

    double halfDepth =
        (touching == RobotSide.FRONT || touching == RobotSide.BACK) ? lengthM / 2 : widthM / 2;
    Translation2d center =
        tag.getTranslation().plus(out.times(faceGapM + halfDepth)).plus(side.times(lateralM));

    // Direction the robot's front points
    Rotation2d towardTag = normal.plus(Rotation2d.k180deg);
    Rotation2d heading =
        switch (touching) {
          case FRONT -> towardTag;
          case BACK -> normal;
          case LEFT -> towardTag.minus(Rotation2d.kCCW_90deg); // left side faces tag
          case RIGHT -> towardTag.plus(Rotation2d.kCCW_90deg);
        };
    return new Pose2d(center, heading);
  }
}
