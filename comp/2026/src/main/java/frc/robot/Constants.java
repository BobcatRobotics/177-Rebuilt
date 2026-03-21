// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.bobcatrobotics.Util.Interpolators.TripleOutputInterpolator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
// import frc.robot.util.VisionObservation;
// import frc.robot.util.VisionObservation.LLTYPE;

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

        public static final double idleFlywheelSpeedRPS = 0;
        public static final double idleIntakeSpeedRPS = 0;
        public static final double idleHoodSpeedRPS = 0;

        public static final double targetFlywheelSpeedRPS = 56;
        public static final double targetIntakeSpeedRPS = 75;
        public static final double targetHoodSpeedRPS = 20;

        public static final double[] targetDistances = {40,50,60,70,80,90,100,110};
        public static final double[] targetFlywheelSpeedsInRPS = {70,66,61,49,45,45,45,45};
        public static final double[] targetIntakeSpeedInRPS = {80,80,80,80,80,80,80,80};
        public static final double[] targetHoodSpeedInRPS = {-40,-35,-25,4,16,20,22,24};


        public final static class SharedFlywheel {
            public static final boolean isInvertedInnerLeft = false;
            public static final boolean isInvertedOuterLeft = true;
            public static final boolean isInvertedInnerRight = false;
            public static final boolean isInvertedOuterRight = false;
            public static final boolean isCoastLeft = true;
            public static final boolean isCoastRight = true;
            // Motor Constants
            public static final double kshooterMainkP = 1.66;
            public static final double kshooterMainkI = 0;
            public static final double kshooterMainkD = 0;
            public static final double kshooterMainkS = 0.303;
            public static final double kshooterMainkV = 0.12;
            public static final double kshooterMainkA = 0;
            public static final double statorCurrentLimit = 80;
            public static final double supplyCurrentLimit = 60;

            public static final int FlywheelOuterIDLeft = 14;
            public static final int FlywheelInnerIDLeft = 13;
            public static final int FlywheelOuterIDRight = 11;
            public static final int FlywheelInnerIDRight = 14; // NOT USED ANYMORE
        }


        public final static class SharedIntake {
            public static final int intakeIDLeft = 15;
            public static final double kIntakeMotorkP = 1.708;
            public static final double kIntakeMotorkI = 0;
            public static final double kIntakeMotorkD = 0;
            public static final double kIntakeMotorkS = 0.375;
            public static final double kIntakeMotorkV = 0.117;
            public static final double kIntakeMotorkA = 0;
            public static final double statorCurrentLimit = 80;
            public static final double supplyCurrentLimit = 60;
            public static final boolean isInverted = false;
            public static final boolean isCoast = false;
        }

        public final static class Left {

            public static final int HoodID = 12;
            // Motor Constants
            public static final double kHoodMotorkP = 1.629;
            public static final double kHoodMotorkI = 0.0;
            public static final double kHoodMotorkD = 0;
            public static final double kHoodMotorkS = 0.301;
            public static final double kHoodMotorkV = 0.123;
            public static final double kHoodMotorkA = 0;
            public static final double statorCurrentLimit = 60;
            public static final double supplyCurrentLimit = 40;
            public static final boolean isInverted = true;
            public static final boolean isCoast = true;

        }

        public final static class Right {
            // ID Constants
            public static final int HoodID = 17;
            public static final double kHoodMotorkP = 1.629;
            public static final double kHoodMotorkI = 0.0;
            public static final double kHoodMotorkD = 0;
            public static final double kHoodMotorkS = 0.301;
            public static final double kHoodMotorkV = 0.123;
            public static final double kHoodMotorkA = 0;
            public static final double statorCurrentLimit = 60;
            public static final double supplyCurrentLimit = 40;
            public static final boolean isInverted = false;
            public static final boolean isCoast = true;
        }

        public final static class ValuesOfKnownShots{
            public static final double[] distance = {1.016, 1.27, 1.524, 1.778, 2.032, 2.286, 2.54, 2.794}; //distance from front LL to center of hub (meters)
            public static final double[] carwashSpeed ={80,80,80,80,80,80,80,80}; //RPS
            public static final double[] mainFlyWheelSpeed = {70,66,61,49,45,45,45,45}; //RPS
            public static final double[] hoodSpeed = {-40,-35,-25,4,16,20,22,24}; //RPS
        }
    }

    public static final class HopperConstants {
        public static final class Top {
            public static final boolean isInverted = false;
            public static final boolean isCoast = true;
            public static final double kHopperP = 2.755;
            public static final double kHopperI = 0;
            public static final double kHopperD = 0.1;
            public static final double kHopperV = 0.073;
            public static final double kHopperS = 0.739;
            public static final double kHopperA = 0;
            public static final double hopperCurrentLimit = 40;
            public static final int hopperMotorId = 16;
        }

        public static final class Bottom {
            public static final boolean isInverted = false;
            public static final boolean isCoast = true;
            public static final double kHopperP = 2.755;
            public static final double kHopperI = 0;
            public static final double kHopperD = 0.1;
            public static final double kHopperV = 0.073;
            public static final double kHopperS = 0.739;
            public static final double kHopperA = 0;
            public static final double hopperCurrentLimit = 40;
            public static final int hopperMotorId = 16;
        }

        public static final double idleHopperSpeed = 0.0;
        public static final double topMotorTargetVelocity = 90;
        public static final double bottomMotorTargetVelocity = 90;

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
//UPDATE ID
            public static int rollerMotorId = 19;
        }
//Original motor
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