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

          // hub target location (from blue side origin/FRC WPIBlue origin) in meters
          public static final double HUBX = 4.7;
          public static final double HUBY = 4.114;

          // left side blue target location (from blue side origin/FRC WPIBlue origin) in meters
          public static final double LEFTBLUEX = 0.1;
          public static final double LEFTBLUEY = 4.114;

    }

    public static final class frontLimelightConstants{
      public static String name = "limelight-front";
      public static LLTYPE limelightType = LLTYPE.LL4;
      public static double verticalFOV = 55.781; //degrees
      public static double horizontalFOV = 81.253; //degrees
      public static double limelightMountHeight = .84;
     // public static int detectorPiplineIndex = 1;
      public static int apriltagPipelineIndex = 0;
    //   int horPixels
    //   Vector<N3> visionMeasurementStdDevs


    }

    public static final class backLimelightConstants{
      public static String name = "limelight-back";
      public static LLTYPE limelightType = LLTYPE.LL4;
      public static double verticalFOV = 0;
      public static double horizontalFOV = 0; 
      public static double limelightMountHeight = .84;
    //   int detectorPiplineIndex
      public static int apriltagPipelineIndex = 0;
    //   int horPixels
    //   Vector<N3> visionMeasurementStdDevs


    }
}


