package frc.robot.subsystems.Intake;

public class IntakeConstants {
    public static double idleIntakePosition = 0;
    public static double targetIntakePosition = 0;
    public static double idleRollerSpeed = 0;
    public static double targetIntakeSpeed = 20;


    public final class PivotConstants {
        public static final boolean isInverted = true;
        public static final boolean isCoast = false;
        public static double kP = 1.1;
        public static double kI = 0.00;
        public static double kD = 0.00;
        public static double kV = 0.00;
        public static double kS = 0.00;
        public static double kA = 0.00;
        public static double SUPPLY_CURRENT_LIMIT = 80;
        public static double STATOR_CURRENT_LIMIT = 80;
        public static int pivotMotorId = 10;
        public static double peakForwardLimit = 90;
        public static double peakReverseLimit = -90;
        public static double ANGLE_TOLERANCE_DEGREES = 2.0;
    }


    public  final class RightRollerConstants {
        public static final boolean isInverted = false;
        public static final boolean isCoast = true;
        public static double kP = 1.628; //kP based on test was 1.995
        public static double kI = 0.00;
        public static double kD = 0.00;
        public static double kV = 0.123;
        public static double kS = 0.312;
        public static double kA = 0.00;
        public static double SUPPLY_CURRENT_LIMIT = 60;
        public static double STATOR_CURRENT_LIMIT = 80;
        public static double peakForwardLimit = 90;
        public static double peakReverseLimit = -90;
        public static int rollerMotorId = 19;
    }
    //Original motor
    public  final class LeftRollerConstants {
        public static final boolean isInverted = true;
        public static final boolean isCoast = true;
        public static double kP = 1.628; //kP based on test was 1.995
        public static double kI = 0.00;
        public static double kD = 0.00;
        public static double kV = 0.123;
        public static double kS = 0.312;
        public static double kA = 0.00;
        public static double SUPPLY_CURRENT_LIMIT = 60;
        public static double STATOR_CURRENT_LIMIT = 80;
        public static double peakForwardLimit = 90;
        public static double peakReverseLimit = -90;

        public static int rollerMotorId = 18;
    }
}