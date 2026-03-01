// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

    public static final class ClimberConstants{
            public static final boolean isInverted = false;
            public static final boolean isCoast = true;
            public static final double kClimberP = 0;
            public static final double kClimberI = 0;
            public static final double kClimberD = 0;
            public static final double kClimberV = 0;
            public static final double kClimberS = 0;
            public static final double kClimberA = 0;
            public static final double climberCurrentLimit = 40;
            public static final int climberMotorId = 11;
            public static double idleClimberPosition = 0.0;
            public static double manualClimberPosition = 0.0;
            public static double targetClimberPosition = 1.5; 
    }
}
