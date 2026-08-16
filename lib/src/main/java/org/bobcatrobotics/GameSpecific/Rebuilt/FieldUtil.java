package org.bobcatrobotics.GameSpecific.Rebuilt;

import org.bobcatrobotics.GameSpecific.Rebuilt.RebuiltFieldConstants.AprilTagLayoutType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldUtil {
      public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

  
    public static Pose2d toNewFieldCoordinate(Pose2d pose2026) {
    Translation2d translation = new Translation2d(
        fieldLength / 2.0 - pose2026.getX(),
        pose2026.getY() - fieldWidth / 2.0
    );

    Rotation2d rotation = pose2026.getRotation()
        .plus(Rotation2d.fromDegrees(180.0));

    return new Pose2d(translation, rotation);
}
}
