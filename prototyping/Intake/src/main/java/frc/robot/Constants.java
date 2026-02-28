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

    public static final class IntakeConstants {
        public static double idleIntakePosition = 0.0;
        public static double targetIntakePosition = 0;
        public static double idleRollerSpeed = 0;
        public static double targetIntakeSpeed = 0;

        public static final class PivotConstants {
            public static final boolean isInverted = false;
            public static final boolean isCoast = false;
            public static double kP = 0.00;
            public static double kI = 0.00;
            public static double kD = 0.00;
            public static double kV = 0.00;
            public static double kS = 0.00;
            public static double kA = 0.00;
            public static double currentLimit = 10;
            public static int pivotMotorId = 10;
        }

        public static final class RollerConstants {
            public static final boolean isInverted = false;
            public static final boolean isCoast = true;
            public static double kP = 0.00;
            public static double kI = 0.00;
            public static double kD = 0.00;
            public static double kV = 0.00;
            public static double kS = 0.00;
            public static double kA = 0.00;
            public static double currentLimit = 10;

            public static int rollerMotorId = 10;
        }

    }
}
