// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
        public static final double idleFlywheelSpeedRPS = 50;
        public static final double idleIntakeSpeedRPS = 30;
        public static final double idleBackspinSpeedLeftRPS = -20;
        public static final double idleBackspinSpeedRightRPS = -20;


        public static final double targetFlywheelSpeedRPS = 50;
        public static final double targetIntakeSpeedRPS = 30;
        public static final double targetBackspinSpeedLeftRPS = -20;
        public static final double targetBackspinSpeedRightRPS = -20;

        public final static class SharedFlywheel {
            public static final boolean isInvertedLeft = true;
            public static final boolean isInvertedRight = false;
            public static final boolean isCoastLeft = true;
            public static final boolean isCoastRight = true;
            // Motor Constants
            public static final double kshooterMainkP = 1.25;
            public static final double kshooterMainkI = 0;
            public static final double kshooterMainkD = 0;
            public static final double kshooterMainkS = 0;
            public static final double kshooterMainkV = 0.0;
            public static final double kshooterMainkA = 0;
            public static final double currentLimit = 40;

            public static final int FlywheelOuterIDLeft = 16;
            public static final int FlywheelInnerIDLeft = 13;
            public static final int FlywheelOuterIDRight = 17;
            public static final int FlywheelInnerIDRight = 14;
        }

        public final static class SharedIntake{
            public static final int intakeIDLeft = 21;
            public static final double kIntakeMotorkP = 1;
            public static final double kIntakeMotorkI = 0;
            public static final double kIntakeMotorkD = 0;
            public static final double kIntakeMotorkS = 0;
            public static final double kIntakeMotorkV = 0;
            public static final double kIntakeMotorkA = 0;
            public static final double currentLimit = 40;
            public static final boolean isInverted = false;
            public static final boolean isCoast = true;
        }

        public final static class Left {

            public static final int BackspinID = 18;
            // Motor Constants
            public static final double kBackspinMotorkP = 2;
            public static final double kBackspinMotorkI = 0;
            public static final double kBackspinMotorkD = 0;
            public static final double kBackspinMotorkS = 0;
            public static final double kBackspinMotorkV = 0;
            public static final double kBackspinMotorkA = 0;
            public static final double currentLimit = 40;
            public static final boolean isInverted = true;
            public static final boolean isCoast = true;

        }

        public final static class Right {
            // ID Constants
            public static final int BackspinID = 12;
            public static final double kBackspinMotorkP = 2;
            public static final double kBackspinMotorkI = 0;
            public static final double kBackspinMotorkD = 0;
            public static final double kBackspinMotorkS = 0;
            public static final double kBackspinMotorkV = 0;
            public static final double kBackspinMotorkA = 0;
            public static final double currentLimit = 40;
            public static final boolean isInverted = false;
            public static final boolean isCoast = true;
        }
    }
}
