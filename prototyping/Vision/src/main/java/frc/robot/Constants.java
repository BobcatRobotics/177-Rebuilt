// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

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

        // lines
        public static double BLUE_LINE = 3.977894;
        public static double RED_LINE = 12.664694;

        // center of hub locations
        public static double BLUE_HUB_X = 4.625594;
        public static double BLUE_HUB_Y = 4.034663;
        public static double RED_HUB_X = 11.915394;
        public static double RED_HUB_Y = 4.034663;
        
        // center line y
        public static double CENTER_Y = 4.034663;

        // passing points
        // TODO add passing points
        public static double BLUE_PASSING_1_X = 1.988947;
        public static double BLUE_PASSING_1_Y = 6.0519945;
        public static double BLUE_PASSING_2_X = 1.988947;
        public static double BLUE_PASSING_2_Y = 2.0173315;
        public static double RED_PASSING_1_X = 14.552041;
        public static double RED_PASSING_1_Y = 6.0519945;
        public static double RED_PASSING_2_X = 14.552041;
        public static double RED_PASSING_2_Y = 2.0173315;
    }
}
