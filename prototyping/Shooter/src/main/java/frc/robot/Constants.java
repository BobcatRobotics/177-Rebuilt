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
        public static final InvertedValue bottomRightMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final NeutralModeValue bottomRightMotorBrakeMode = NeutralModeValue.Coast;
        public static final double kBottomRightP = 7.5;
        public static final double kBottomRightS = 0;
        public static final double kBottomRightV = 0.25;
        public static final double bottomRightCurrentLimit = 40;

        public static final InvertedValue bottomMotorLeftInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue bottomMotorLeftBrakeMode = NeutralModeValue.Coast;
        public static final double kBottomLeftP = 7.5;
        public static final double kBottomLeftS = 0;
        public static final double kBottomLeftV = 0.25;
        public static final double bottomLeftCurrentLimit = 40;
        public static final double idlePosition = 0.0;
        public static final double idleSpeed = 0.0;

        public static final InvertedValue topTopMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue topTopMotorBrakeMode = NeutralModeValue.Coast;
        public static final double kTopTopP = 5;
        public static final double kTopTopS = 0;
        public static final double kTopTopV = 0;
        public static final double topTopCurrentLimit = 40;

        public static final InvertedValue topBottomMotorInvert = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue topBottomMotorBrakeMode = NeutralModeValue.Coast;
        public static final double kTopBottomP = 5;
        public static final double kTopBottomS = 0;
        public static final double kTopBottomV = 0;
        public static final double topBottomCurrentLimit = 40;

    }
}
