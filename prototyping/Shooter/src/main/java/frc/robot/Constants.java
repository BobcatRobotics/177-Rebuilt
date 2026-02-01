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
        public static final double idleFlywheelSpeedRPS = 0;
        public static final double idleIntakeSpeedRPS = 0;
        public static final double idleBackspinSpeedRPS = 0;

        public static final class LeftShooterConstants {
            public static final double targetFlywheelSpeedRPS = 52.5;
            public static final double targetIntakeSpeedRPS = 0.2;
            public static final double targetBackspinSpeedRPS = 30;
            // ID Constants
            public static final int FlywheelIDLeft = 29;
            public static final int BackspinIDLeft = 2;
            public static final int intakeIDLeft = 30;
            // Stator Current Limits
            public static final double shooterMainMotorCurrentLimit = 40;
            public static final double shooterIntakeMotorCurrentLimit = 80;
            // Main Flywheel Motor Settings
            public static final boolean shooterMainMotorLeftMountedInvertState = true;
            public static final boolean shooterMainMotorRightMountedInvertState = false;
            public static final boolean shooterMainMotorLeftMountedIsCoast = true;
            public static final boolean shooterMainMotorRightMountedIsCoast = true;
            public static final double kshooterMainkP = 1.25;
            public static final double kshooterMainkI = 0;
            public static final double kshooterMainkD = 0.0165;
            public static final double kshooterMainkS = 0;
            public static final double kshooterMainkV = 0;
            public static final double kshooterMainkA = 0;
            // Backspin Motor Settings
            public static final double shooterBackspinMotorCurrentLimit = 40;
            // -> Use only if the motor is mounted on left side of the shooter
            public static final boolean shooterBackspinMotorLeftMountedInvertedState = true;
            public static final boolean shooterBackspinMotorLeftMountedIsCoast = true;
            public static final double shooterBackspinMotorLeftMountedkP = 5;
            public static final double shooterBackspinMotorLeftMountedkI = 0;
            public static final double shooterBackspinMotorLeftMountedkD = 0;
            public static final double shooterBackspinMotorLeftMountedkS = 0;
            public static final double shooterBackspinMotorLeftMountedkV = 0;
            public static final double shooterBackspinMotorLeftMountedkA = 0;
            // -> Use only if the motor is mounted on right side of the shooter
            public static final boolean shooterBackspinMotorRightMountedIsCoast = true;
            public static final boolean shooterBackspinMotorRightMountedInvertedState = false;
            public static final double shooterBackspinMotorRightMountedkP = 5;
            public static final double shooterBackspinMotorRightMountedkI = 0;
            public static final double shooterBackspinMotorRightMountedkD = 0;
            public static final double shooterBackspinMotorRightMountedkS = 0;
            public static final double shooterBackspinMotorRightMountedkV = 0;
            public static final double shooterBackspinMotorRightMountedkA = 0;
            // Intake Motor Settings
            public static final double shooterIntakeMotorkP = 5;
            public static final double shooterIntakeMotorkI = 0;
            public static final double shooterIntakeMotorkD = 0;
            public static final double shooterIntakeMotorkS = 0;
            public static final double shooterIntakeMotorkV = 0;
            public static final double shooterIntakeMotorkA = 0;
            // -> Use only if the motor is mounted on left side of the shooter
            public static final boolean shooterIntakeMotorLeftMountedInvertedState = true;
            public static final boolean shooterIntakeMotorLeftMountedIsCoast = true;
            // -> Use only if the motor is mounted on right side of the shooter
            public static final boolean shooterIntakeMotorRightMountedIsCoast = true;
            public static final boolean shooterIntakeMotorRightMountedInvertedState = false;
        }

        public static final class RightShooterConstants {
            public static final double targetFlywheelSpeedRPS = 52.5;
            public static final double targetIntakeSpeedRPS = 0.2;
            public static final double targetBackspinSpeedRPS = 30;
            // ID Constants
            public static final int FlywheelIDRight = 17;
            public static final int BackspinIDRight = 20;
            public static final int intakeIDRight = 28;
            // Stator Current Limits
            public static final double shooterMainMotorCurrentLimit = 40;
            public static final double shooterIntakeMotorCurrentLimit = 80;
            public static final double shooterBackspinMotorCurrentLimit = 40;
            // Main Flywheel Motor Settings
            public static final boolean shooterMainMotorLeftMountedInvertState = true;
            public static final boolean shooterMainMotorRightMountedInvertState = false;
            public static final boolean shooterMainMotorLeftMountedIsCoast = true;
            public static final boolean shooterMainMotorRightMountedIsCoast = true;
            public static final double kshooterMainkP = 1.25;
            public static final double kshooterMainkI = 0;
            public static final double kshooterMainkD = 0.0165;
            public static final double kshooterMainkS = 0;
            public static final double kshooterMainkV = 0;
            public static final double kshooterMainkA = 0;
            // Backspin Motor Settings
            public static final double shooterBackspinMotorkP = 5;
            public static final double shooterBackspinMotorkI = 0;
            public static final double shooterBackspinMotorkD = 0;
            public static final double shooterBackspinMotorkS = 0;
            public static final double shooterBackspinMotorkV = 0;
            public static final double shooterBackspinMotorkA = 0;
            // -> Use only if the motor is mounted on left side of the shooter
            public static final boolean shooterBackspinMotorLeftMountedInvertedState = true;
            public static final boolean shooterBackspinMotorLeftMountedIsCoast = true;
            // -> Use only if the motor is mounted on right side of the shooter
            public static final boolean shooterBackspinMotorRightMountedIsCoast = true;
            public static final boolean shooterBackspinMotorRightMountedInvertedState = false;
            // Intake Motor Settings
            public static final double shooterIntakeMotorkP = 5;
            public static final double shooterIntakeMotorkI = 0;
            public static final double shooterIntakeMotorkD = 0;
            public static final double shooterIntakeMotorkS = 0;
            public static final double shooterIntakeMotorkV = 0;
            public static final double shooterIntakeMotorkA = 0;
            // -> Use only if the motor is mounted on left side of the shooter
            public static final boolean shooterIntakeMotorLeftMountedInvertedState = true;
            public static final boolean shooterIntakeMotorLeftMountedIsCoast = true;
            // -> Use only if the motor is mounted on right side of the shooter
            public static final boolean shooterIntakeMotorRightMountedIsCoast = true;
            public static final boolean shooterIntakeMotorRightMountedInvertedState = false;
        }
    }
}
