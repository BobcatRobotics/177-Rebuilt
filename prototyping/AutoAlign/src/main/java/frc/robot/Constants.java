// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.VisionObservation;
import frc.robot.util.VisionObservation.LLTYPE;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final class FieldConstants {
        public static final double fieldLengthInches = 651.22;
        public static final double fieldWidthInches = 317.69;

         public static final double aprilTagWidth = Units.inchesToMeters(6.50);
         public static final int aprilTagCount = 32;
    }

    public static final class LimelightConstants{
      public static String name = "";
      public static LLTYPE limelightType = LLTYPE.LL4;
    //   double verticalFOV
    //   double horizontalFOV 
      public static double limelightMountHeight = .84;
    //   int detectorPiplineIndex
      public static int apriltagPipelineIndex = 0;
    //   int horPixels
    //   Vector<N3> visionMeasurementStdDevs


    }
}


