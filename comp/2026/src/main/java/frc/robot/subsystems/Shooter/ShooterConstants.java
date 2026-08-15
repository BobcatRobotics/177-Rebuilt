package frc.robot.subsystems.Shooter;

public final class ShooterConstants {

    public static final double SPEED_TOLERANCE_RPS = 2.0;
    public static final double idleDumperSpeed = 0;
    public static final double idleHoodPosition = 0;
    // who knows what distance
    public static final double targetDumperSpeed = 50;
    public static final double targetHoodPosition = -5.5;

    public static final double motionMagicCruiseVelocity = 190;
    public static final double motionMagicAcceleration = 175;
    public static final double motionMagicJerk = 0;
    public static final double motionMagicExpoKV = 0.0;
    public static final double motionMagicExpoKa = 0.01;

    public final static class Left {
        // public static final int HoodID = 12;
        public static final int dumperLeftUpID = 13;
        public static final int dumperLeftDownID = 20;
        // Motor Constants
        public static final double kdumperLeftMotorkP = 0.19;
        public static final double kdumperLeftMotorkI = 0.0;
        public static final double kdumperLeftMotorkD = 0;
        public static final double kdumperLeftMotorkS = 0.18;
        public static final double kdumperLeftMotorkV = 0.13;
        public static final double kdumperLeftMotorkA = 0.4;
        public static final double statorCurrentLimit = 80;
        public static final double supplyCurrentLimit = 60;
        public static final boolean isInverted = false;
        public static final boolean isCoast = true;
        public static final boolean isSoftLimitsEnabled = false;
        public static final boolean useMotionMagic = true;

        public static final double motionMagicCruiseVelocity = 190;
        public static final double motionMagicAcceleration = 600;
        public static final double motionMagicJerk = 0;
        public static final double motionMagicExpoKV = 0.0;
        public static final double motionMagicExpoKa = 0.0;
    }

    public final static class Right {
        // ID Constants
        // public static final int HoodID = 17;
        public static final int dumperRightUpID = 11;
        public static final int dumperRightDownID = 14;
        // Motor Constants
        public static final double kdumperRightMotorkP = 0.19;
        public static final double kdumperRightMotorkI = 0.0;
        public static final double kdumperRightMotorkD = 0;
        public static final double kdumperRightMotorkS = 0.18;
        public static final double kdumperRightMotorkV = 0.13;
        public static final double kdumperRightMotorkA = 0.4;
        public static final double statorCurrentLimit = 80;
        public static final double supplyCurrentLimit = 60;
        public static final boolean isInverted = true;
        public static final boolean isCoast = true;
        public static final boolean isSoftLimitsEnabled = false;
        public static final boolean useMotionMagic = true;

    }

    public final static class ValuesOfKnownShots {
        public static final double offsetDistanceInMeters = 0.92837;
        public static final double[] distance = { 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160,
                170.0 }; // Inches from bumper vertex to hub face
        public static final double[] carwashSpeed = { 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80 }; // RPS
        public static final double[] dumperSpeed = { 30, 30, 32, 32.5, 33.5, 33, 34.5, 34.5, 35, 35.5, 36, 36.5, 37.25,
                37.75, 39, 39.0 }; // RPS
        public static final double[] hoodPosition = { 0, 0, -0.3, -0.6, -0.85, -1.4, -1.8, -2.175, -2.25, -2.5, -2.9,
                -3.2, -3.3, -3.6, -4, -4.2 }; // Degrees/Position
    }

    public final static class PassingValuesOfKnownShots {
        public static final double offsetDistanceInMeters = 0.92837;
        public static final double[] distance = { 134, 238, 324, 420, 579 }; // Inches from bumper vertex to hub face
        public static final double[] carwashSpeed = { 80, 80, 80, 80, 80 }; // RPS
        public static final double[] dumperSpeed = { 30, 37, 50, 57, 75 }; // RPS
        public static final double[] hoodPosition = { -5.5, -5.5, -5.9, -6.2, -6.2 }; // Degrees/Position
    }
}
