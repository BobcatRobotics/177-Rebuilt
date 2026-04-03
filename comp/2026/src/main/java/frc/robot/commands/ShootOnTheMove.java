package frc.robot.commands;

import org.bobcatrobotics.GameSpecific.Rebuilt.HubUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;
import org.bobcatrobotics.Util.Interpolators.TripleOutputInterpolator;

public class ShootOnTheMove{
    private static final double HOOD_ANGLE_RAD = Math.toRadians(45.0); // Need to get

    // Time of flight map to get better future pose value
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = 
        new InterpolatingDoubleTreeMap();
    
    static {
        // distance (meters) -> time (seconds)
        timeOfFlightMap.put(0.0,0.0); // add values
    }

    private static final TripleOutputInterpolator shooterInterpolator = new TripleOutputInterpolator(
        new double[]{},  // add values
        new double[]{},  // add values
        new double[]{},  // add values
        new double[]{},  // add values
        true
    );

    public static TripleOutputInterpolator.Speeds calculateRequiredSpeeds(Pose2d robotPose, ChassisSpeeds robotSpeed) {
        //Initial Target Vector
        Translation2d goalLocation = HubUtil.getMyHubCoordinates(RobotState.getInstance().alliance).toPose2d().getTranslation();
        Translation2d robotPos = robotPose.getTranslation();
        double initialDistance = goalLocation.getDistance(robotPos);
        
        //Lookahead Calculations
        Translation2d futurePos = robotPos;
        double futureDistance = initialDistance;
        double timeOfFlight = timeOfFlightMap.get(initialDistance);
        
        //Iterating to get accurate shot
        for (int i = 0; i < 20; i++) {
            //Get time of flight for current distance estimate
            timeOfFlight = timeOfFlightMap.get(futureDistance);
            
            //Calculate where robot will be when note arrives
            double offsetX = robotSpeed.vxMetersPerSecond * timeOfFlight;
            double offsetY = robotSpeed.vyMetersPerSecond * timeOfFlight;
            futurePos = robotPos.plus(new Translation2d(offsetX, offsetY));
            
            //Recalculate distance from future position
            futureDistance = goalLocation.getDistance(futurePos);
        }
        
        //Target Vector from future position
        Translation2d targetVec = goalLocation.minus(futurePos);
        Rotation2d targetAngle = targetVec.getAngle();
        
        //Components of robot velocity
        Translation2d robotVelVec = new Translation2d(
            robotSpeed.vxMetersPerSecond, 
            robotSpeed.vyMetersPerSecond
        );
        
        //Velocity component along the shot direction
        double velocityAlongShot = robotVelVec.getX() * Math.cos(targetAngle.getRadians()) 
                                  + robotVelVec.getY() * Math.sin(targetAngle.getRadians());
        
        //Base speeds at future position
        TripleOutputInterpolator.Speeds baseSpeeds = shooterInterpolator.get(futureDistance);
        
        //Horizontal velocity component of fuel
        double baseHorizontalSpeed = baseSpeeds.one * Math.cos(HOOD_ANGLE_RAD);
        
        //Adjust for robot velocity along shot direction
        double requiredHorizontalSpeed = baseHorizontalSpeed - velocityAlongShot;
        
        //Scale factor based on initial stationary speeds
        double scaleFactor = requiredHorizontalSpeed / baseHorizontalSpeed;
        
        //Scaled speeds
        return new TripleOutputInterpolator.Speeds(
            baseSpeeds.one * scaleFactor,
            baseSpeeds.two * scaleFactor,
            baseSpeeds.three * scaleFactor
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