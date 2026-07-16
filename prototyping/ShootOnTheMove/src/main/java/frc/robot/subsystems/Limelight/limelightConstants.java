package frc.robot.subsystems.Limelight;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.VisionObservation.LLTYPE;

public class limelightConstants {
  public final String name;
  public final LLTYPE limelightType;
  // public final double verticalFOV;
  // public final double horizontalFOV;
  public final double limelightMountHeight;
  // public final int detectorPiplineIndex;
  public final int apriltagPipelineIndex;
  // public final int horPixels;
  // public final Vector<N3> visionMeasurementStdDevs;

  /**
   * Limelight Constants to be used when creating swerve modules.
   *
   * @param name the limelight name
   * @param limelightType the limelight hardware/type
   * @param limelightMountHeight the height the limelight is mounted at (meters)
   * @param apriltagPipelineIndex the default pipeline index for april tag detection
   */
  public limelightConstants(
      String name,
      LLTYPE limelightType,
      // double verticalFOV,
      // double horizontalFOV,
      double limelightMountHeight,
      // int detectorPiplineIndex,
      int apriltagPipelineIndex
      ) {
    this.name = name;
    // this.verticalFOV = verticalFOV; // degrees obviously
    // this.horizontalFOV = horizontalFOV;
    this.limelightMountHeight = 0.84;
    // this.detectorPiplineIndex = detectorPiplineIndex;
    this.apriltagPipelineIndex = apriltagPipelineIndex;
    // this.horPixels = horPixels;
    // this.visionMeasurementStdDevs = visionMeasurementStdDevs;
    this.limelightType = limelightType;
  }
}

// enum LLTYPE {
//   LL3,
//   LL3G,
//   LL4
// }
