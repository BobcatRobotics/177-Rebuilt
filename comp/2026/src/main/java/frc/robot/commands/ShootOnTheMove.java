package frc.robot.commands;

import org.bobcatrobotics.GameSpecific.Rebuilt.HubUtil;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;

import org.bobcatrobotics.Util.Interpolators.TripleOutputInterpolator;

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
        double mainFlyWheelSpeedRPM  = RobotState.getInstance().interpolator.getAsList(initialDistance).get(0);
        
        //Lookahead Calculations
        double mainFlyWheelSpeedMS = mainFlyWheelSpeedRPM * (2*Math.PI*0.0508); //0.0508 is flywheel radius in meters
        double vSinTheta = mainFlyWheelSpeedMS * Math.sin(HOOD_ANGLE_RAD);
        double timeOfFlight = (vSinTheta + Math.sqrt(vSinTheta * vSinTheta + 2 * 9.81 * 1.3912)) / 9.81;
        Translation2d futurePos = new Translation2d(
            robotPos.getX() + robotSpeed.vxMetersPerSecond * timeOfFlight,
            robotPos.getY() + robotSpeed.vyMetersPerSecond * timeOfFlight
        );
        double futureDistance = goalLocation.getDistance(futurePos);

        //Base speeds at future position
        double adjustedMainRPM = RobotState.getInstance().interpolator.getAsList(futureDistance).get(0);
        double adjustedHoodRPM = RobotState.getInstance().interpolator.getAsList(futureDistance).get(2);
        
        //Projecting robot velocity onto the shot direction
        double shotDirX = (goalLocation.getX() - robotPos.getX()) / initialDistance;
        double shotDirY = (goalLocation.getY() - robotPos.getY()) / initialDistance;
 
        //Changing speeds based on direction of shot
        double rawVelocityAlongShot =
            robotSpeed.vxMetersPerSecond * shotDirX +
            robotSpeed.vyMetersPerSecond * shotDirY;
 
        double velocityAlongShot = velocityFilter.calculate(rawVelocityAlongShot);
 
        //Scale motors
        double adjustedMainMS = adjustedMainRPM * (2 * Math.PI * 0.0508) / 60.0;
        double baseHorizontalSpeed = adjustedMainMS * Math.cos(HOOD_ANGLE_RAD);
        double scaleFactor = (baseHorizontalSpeed - velocityAlongShot) / baseHorizontalSpeed;
 
        //Clamp to prevent absurd values if driving very fast
        scaleFactor = Math.max(0.5, Math.min(1.5, scaleFactor)); //Tune clamp range
 
        return new TripleOutputInterpolator.Speeds(
            adjustedMainRPM * scaleFactor,
            80,
            adjustedHoodRPM * scaleFactor
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