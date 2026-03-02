package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

// util class for aiming to correct hub and passing to the correct location
public final class AimingUtil {

    // getting alliance
    static Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    static boolean isRed = false;//alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    static boolean isBlue = true; //alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue;

    private AimingUtil() {}

    // returns rotation to the "valid" hub
    public static double RotationToHub(Pose2d robotPose) {
        double rotationRad = 0;
        // if on the blue side, not neutral zone
        if ((robotPose.getX() < Constants.FieldConstants.BLUE_LINE) && isBlue) {
            rotationRad = Math.atan2(
                Constants.FieldConstants.BLUE_HUB_Y - robotPose.getY(),
                Constants.FieldConstants.BLUE_HUB_X - robotPose.getX()
            );
        }
        // if on the red side, not neutral zone
        else if ((robotPose.getX() > Constants.FieldConstants.RED_LINE) && isRed) {
            rotationRad = Math.atan2(
                Constants.FieldConstants.RED_HUB_Y - robotPose.getY(),
                Constants.FieldConstants.RED_HUB_X - robotPose.getX()
            );
        }
        // assume neutral zone
        else {
            System.out.println("No valid zone");
        }
        return rotationRad;
    }

    // returns rotation to closest passing shot location
    public static double RotationToPass(Pose2d robotPose) {
        double rotationRad = 0;
        // if left from blue side, aim to point 1
        if ((robotPose.getY() > Constants.FieldConstants.CENTER_Y) && isBlue) {
            rotationRad = Math.atan2(
                Constants.FieldConstants.BLUE_PASSING_1_Y - robotPose.getY(),
                Constants.FieldConstants.BLUE_PASSING_1_X - robotPose.getX()
            );
        }
        // otherwise, shoot to right (blue passing 2)
        else if ((robotPose.getY() < Constants.FieldConstants.CENTER_Y) && isBlue) {
            rotationRad = Math.atan2(
                Constants.FieldConstants.BLUE_PASSING_2_Y - robotPose.getY(),
                Constants.FieldConstants.BLUE_PASSING_2_X - robotPose.getX()
            );
        }
        // red passing 1 (right from left side drivers)
        else if ((robotPose.getY() > Constants.FieldConstants.CENTER_Y) && isRed) {
            rotationRad = Math.atan2(
                Constants.FieldConstants.RED_PASSING_1_Y - robotPose.getY(),
                Constants.FieldConstants.RED_PASSING_1_X - robotPose.getX()
            );
        }
        // red passing 2 (left from red)
        else if ((robotPose.getY() < Constants.FieldConstants.CENTER_Y) && isRed) {
            rotationRad = Math.atan2(
                Constants.FieldConstants.RED_PASSING_2_Y - robotPose.getY(),
                Constants.FieldConstants.RED_PASSING_2_X - robotPose.getX()
            );
        }

        else {
            System.out.println("No valid zone");
        }
        return rotationRad;
    }
}
