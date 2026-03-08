// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

public class AprilTagVisionConstants {

  public class limelightConstants {
    public static final double rotationTolerance = 10000;
    public static final double throwoutDist = 4.5;
    public static final double xySingleTagStdDev = 0.6;
    public static final double thetaSingleTagStdDev = 9999999;
    public static final double xyMultiTagStdDev = 0.45;
    public static final double thetaMultiTagStdDev = 99999999;

    public static final double multiFunctionConstant = 0.2;

    public static final int[] validTags =
        new int[] {19,20,21,24,25,26,18,27};
  }
}
