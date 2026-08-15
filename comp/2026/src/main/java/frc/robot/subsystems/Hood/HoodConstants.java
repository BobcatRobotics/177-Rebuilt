package frc.robot.subsystems.Hood;

public final  class HoodConstants {
        public static final boolean isInverted = false;
        public static final boolean isCoast = false;
        // Motor Constants
        public static final double kP = 12;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.25;
        public static final double kV = 0.2;
        public static final double kA = 0;
        public static final double statorCurrentLimit = 80;
        public static final double supplyCurrentLimit = 80;
        public static final int ID = 21;
        public static final boolean isSoftLimitsEnabled = true;
        public static final double forwardSoftwareLimit = -6.2;
        public static final double reverseSoftwareLimit = 0;
        public static final boolean useMotionMagic = true;
        public static final double motionMagicCruiseVelocity = 0;
        public static final double motionMagicAcceleration = 700;
        public static final double motionMagicJerk = 0;
        public static final double motionMagicExpoKV = 0.4;
        public static final double motionMagicExpoKa = 0.01;

        public static final double ANGLE_TOLERANCE_DEGREES = 2.0;
        public static final double GEAR_RATIO = 1.0;
    }
