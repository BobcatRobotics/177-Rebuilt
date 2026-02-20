// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
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

    public final static class ShooterConstants {
        public static final double idleFlywheelSpeedRPS = 0;
        public static final double idleIntakeSpeedRPS = 0;
        public static final double idleBackspinSpeedLeftRPS = 0;
        public static final double idleBackspinSpeedRightRPS = 0;

        public static final double targetFlywheelSpeedRPS = 38.5;
        public static final double targetIntakeSpeedRPS = 78;
        public static final double targetBackspinSpeedLeftRPS = 48.5;
        public static final double targetBackspinSpeedRightRPS = 48.5;

        public final static class SharedFlywheel {
            public static final boolean isInvertedLeft = false;
            public static final boolean isInvertedRight = true;
            public static final boolean isCoastLeft = true;
            public static final boolean isCoastRight = true;
            // Motor Constants
            public static final double kshooterMainkP = 1.49014;
            public static final double kshooterMainkI = 0;
            public static final double kshooterMainkD = 0.000;
            public static final double kshooterMainkS = 0.21405;
            public static final double kshooterMainkV = 0.13422;
            public static final double kshooterMainkA = 0;
            public static final double statorCurrentLimit = 80;
            public static final double supplyCurrentLimit = 60;

            public static final int FlywheelOuterIDLeft = 10;
            public static final int FlywheelInnerIDLeft = 17;
            public static final int FlywheelOuterIDRight = 16;
            public static final int FlywheelInnerIDRight = 14;
        }

        public final static class SharedIntake{
            public static final int intakeIDLeft = 18;
            public static final double kIntakeMotorkP = 2.55996;
            public static final double kIntakeMotorkI = 0;
            public static final double kIntakeMotorkD = 0.00;
            public static final double kIntakeMotorkS = 0.19856;
            public static final double kIntakeMotorkV = 0.12821;
            public static final double kIntakeMotorkA = 0;
            public static final double statorCurrentLimit = 80;
            public static final double supplyCurrentLimit = 60;
            public static final boolean isInverted = true;
            public static final boolean isCoast = true;
        }

        public final static class Left {

            public static final int BackspinID = 28;
            // Motor Constants
            public static final double kBackspinMotorkP = 0.97578;
            public static final double kBackspinMotorkI = 0.0;
            public static final double kBackspinMotorkD = 0.0;
            public static final double kBackspinMotorkS = 0.21264;
            public static final double kBackspinMotorkV = 0.20496;
            public static final double kBackspinMotorkA = 0;
            public static final double statorCurrentLimit = 60;
            public static final double supplyCurrentLimit = 40;
            public static final boolean isInverted = true;
            public static final boolean isCoast = true;

        }

        public final static class Right {
            // ID Constants
            public static final int BackspinID = 2;
            public static final double kBackspinMotorkP = 0.97578;
            public static final double kBackspinMotorkI = 0.0;
            public static final double kBackspinMotorkD = 0.0;
            public static final double kBackspinMotorkS = 0.21264;
            public static final double kBackspinMotorkV = 0.20496;
            public static final double kBackspinMotorkA = 0;
            public static final double statorCurrentLimit = 60;
            public static final double supplyCurrentLimit = 40;
            public static final boolean isInverted = false;
            public static final boolean isCoast = true;
        }
    }
}
