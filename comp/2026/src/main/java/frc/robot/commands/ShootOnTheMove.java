package frc.robot.commands;

import org.bobcatrobotics.GameSpecific.Rebuilt.HubUtil;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;

import org.bobcatrobotics.Util.Interpolators.TripleOutputInterpolator;
import org.littletonrobotics.junction.Logger;

public class ShootOnTheMove{
    private static final double HOOD_ANGLE_RAD = Math.toRadians(65.3); // Need to get
    // private static final double TOF_SECONDS_PER_IN = 0.09; //Random number

    // Time of flight map to get better future pose value
    // private static final InterpolatingDoubleTreeMap timeOfFlightMap = 
    //     new InterpolatingDoubleTreeMap();
    
    // static {
    //     // distance (meters) -> time (seconds)
    //     timeOfFlightMap.put(0.0,0.0); // add values
    // }

    //Filter for rpm to smoothen based on 6328
    private static final LinearFilter velocityFilter = LinearFilter.movingAverage(5);
 
    //Avoiding Stale State
    public static void clearFilter() {
        velocityFilter.reset();
    }

    public static TripleOutputInterpolator.Speeds calculateSpeeds(Drive drive, ChassisSpeeds robotSpeed) {
        //Initial Target Vector
        Translation2d goalLocation = HubUtil.getMyHubCoordinates(RobotState.getInstance().alliance).toPose2d().getTranslation();
        Translation2d robotPos = drive.getPose().getTranslation();
        double initialDistance = goalLocation.getDistance(robotPos);

        //Base speeds at current distance
        double initialDistanceInch = Units.metersToInches(initialDistance);
        double mainFlyWheelSpeedRPS  = RobotState.getInstance().interpolator.getAsList(initialDistanceInch).get(0);
        
        //Lookahead Calculations

        double futureDistance = goalLocation.getDistance(RobotState.getInstance().futurePos.getTranslation());
        Logger.recordOutput("ShootOnTheMove/futureDistance", futureDistance);
        double futureDistanceInch = Units.metersToInches(futureDistance);

        //Base speeds at future position
        double adjustedMainRPS = RobotState.getInstance().interpolator.getAsList(futureDistanceInch).get(2);
        double adjustedHoodRPS = RobotState.getInstance().interpolator.getAsList(futureDistanceInch).get(1);
        
        //Projecting robot velocity onto the shot direction
        double shotDirX = (goalLocation.getX() - robotPos.getX()) / initialDistance;
        double shotDirY = (goalLocation.getY() - robotPos.getY()) / initialDistance;
 
        //Changing speeds based on direction of shot
        double rawVelocityAlongShot =
            robotSpeed.vxMetersPerSecond * shotDirX +
            robotSpeed.vyMetersPerSecond * shotDirY;
 
        double velocityAlongShot = velocityFilter.calculate(rawVelocityAlongShot);
 
        //Scale motors
        double adjustedMainMS = adjustedMainRPS * (2 * Math.PI * 0.0508);
        double baseHorizontalSpeed = adjustedMainMS * Math.cos(HOOD_ANGLE_RAD);
        double scaleFactor = (baseHorizontalSpeed - velocityAlongShot) / baseHorizontalSpeed;
 
        //Clamp to prevent absurd values if driving very fast
        scaleFactor = Math.max(0.5, Math.min(1.5, scaleFactor)); //Tune clamp range
 
        return new TripleOutputInterpolator.Speeds(
            adjustedMainRPS * scaleFactor,
            80,
            adjustedHoodRPS * scaleFactor
        );
    }
    
    public static Rotation2d getTargetRotation(Pose2d robotPose) {
        Translation2d goalLocation = HubUtil.getMyHubCoordinates(RobotState.getInstance().alliance).toPose2d().getTranslation();
        return goalLocation.minus(robotPose.getTranslation()).getAngle();
    }

    
    public static double getDistanceToGoal(Pose2d robotPose) {
        Translation2d goalLocation = HubUtil.getMyHubCoordinates(RobotState.getInstance().alliance).toPose2d().getTranslation();
        return goalLocation.getDistance(robotPose.getTranslation());
    }
}