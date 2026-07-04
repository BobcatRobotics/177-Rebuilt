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
    public static final boolean lowTelemetryMode = true;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public final static class ShooterConstants {
        public final static class ValuesOfKnownShots {
            public static final double offsetDistanceInMeters = 0.92837;
            public static final double[] distance = { 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160,
                    170.0 }; // Inches from bumper vertex to hub face
            public static final double[] carwashSpeed = { 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80,
                    80.0 }; // RPS
            public static final double[] dumperSpeed = { 30, 30, 32, 32.5, 33.5, 33, 34.5, 34.5, 35, 35.5, 36, 36.5,
                    37.25, 37.75, 39, 39.0 }; // RPS
            public static final double[] hoodPosition = { 0, 0, -0.3, -0.6, -0.85, -1.4, -1.8, -2.175, -2.25, -2.5,
                    -2.9, -3.2, -3.3, -3.6, -4, -4.2 }; // Degrees/Position
        }

        public final static class PassingValuesOfKnownShots {
            public static final double offsetDistanceInMeters = 0.92837;
            public static final double[] distance = { 134, 238, 324, 420, 579 }; // Inches from bumper vertex to hub
                                                                                 // face
            public static final double[] carwashSpeed = { 80, 80, 80, 80, 80 }; // RPS
            public static final double[] dumperSpeed = { 30, 37, 50, 57, 75 }; // RPS
            public static final double[] hoodPosition = { -5.5, -5.5, -5.9, -6.2, -6.2 }; // Degrees/Position
        }
    }


    
    public static final class IntakeConstants {
        public static double idleIntakePosition = 0;
        public static double targetIntakePosition = 0;
        public static double idleRollerSpeed = 0;
        public static double targetIntakeSpeed = 20;


        public static final class PivotConstants {
            public static final boolean isInverted = true;
            public static final boolean isCoast = false;
            public static double kP = 1.1;
            public static double kI = 0.00;
            public static double kD = 0.00;
            public static double kV = 0.00;
            public static double kS = 0.00;
            public static double kA = 0.00;
            public static double currentLimit = 80;
            public static int pivotMotorId = 10;
            public static double peakForwardLimit = 90;
            public static double peakReverseLimit = -90;
        }

     
        public static final class RightRollerConstants {
            public static final boolean isInverted = false;
            public static final boolean isCoast = true;
            public static double kP = 1.628; //kP based on test was 1.995
            public static double kI = 0.00;
            public static double kD = 0.00;
            public static double kV = 0.123;
            public static double kS = 0.312;
            public static double kA = 0.00;
            public static double currentLimit = 60;
            public static double peakForwardLimit = 90;
            public static double peakReverseLimit = -90;
            public static int rollerMotorId = 19;
        }
         public static final class LeftRollerConstants {
            public static final boolean isInverted = true;
            public static final boolean isCoast = true;
            public static double kP = 1.628; //kP based on test was 1.995
            public static double kI = 0.00;
            public static double kD = 0.00;
            public static double kV = 0.123;
            public static double kS = 0.312;
            public static double kA = 0.00;
            public static double currentLimit = 60;
            public static double peakForwardLimit = 90;
            public static double peakReverseLimit = -90;

            public static int rollerMotorId = 18;
        }
    }
}
