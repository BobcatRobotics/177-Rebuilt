package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.TripleSpeedInterpolator;

public class ShootOnTheMove{
    private static final double HOOD_ANGLE_RAD = Math.toRadians(45.0); // Need to get

    // Distance limits
    private static final double MIN_DISTANCE = 1.0; // tune this
    private static final double MAX_DISTANCE = 6.0;

    // Time of flight map to get better future pose value
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = 
        new InterpolatingDoubleTreeMap();
    
    static {
        // Add your measured time-of-flight data here
        // Format: distance (meters) -> time (seconds)
        timeOfFlightMap.put(0.0,0.0); // add values
    }

    private static final TripleSpeedInterpolator shooterInterpolator = new TripleSpeedInterpolator(
        new double[]{},  // add values
        new double[]{},  // add values
        new double[]{},  // add values
        new double[]{},  // add values
        true
    );

    public static TripleSpeedInterpolator.Speeds calculateRequiredSpeeds(Pose2d robotPose, ChassisSpeeds robotSpeed) {
        // 1. GET INITIAL TARGET VECTOR
        Translation2d goalLocation = FieldConstants.GOAL_LOCATION;
        Translation2d robotPos = robotPose.getTranslation();
        double initialDistance = goalLocation.getDistance(robotPos);
        
        // 2. ITERATIVE LOOKAHEAD (accounts for robot motion during flight)
        Translation2d futurePos = robotPos;
        double futureDistance = initialDistance;
        double timeOfFlight = timeOfFlightMap.get(initialDistance);
        
        // Iterate to converge on actual shot distance
        for (int i = 0; i < 20; i++) {
            // Get time of flight for current distance estimate
            timeOfFlight = timeOfFlightMap.get(futureDistance);
            
            // Calculate where robot will be when note arrives
            double offsetX = robotSpeed.vxMetersPerSecond * timeOfFlight;
            double offsetY = robotSpeed.vyMetersPerSecond * timeOfFlight;
            futurePos = robotPos.plus(new Translation2d(offsetX, offsetY));
            
            // Recalculate distance from future position
            futureDistance = goalLocation.getDistance(futurePos);
        }
        
        // 3. CALCULATE TARGET VECTOR FROM future POSITION
        Translation2d targetVec = goalLocation.minus(futurePos);
        Rotation2d targetAngle = targetVec.getAngle();
        
        // 4. PROJECT ROBOT VELOCITY ONTO SHOOTING DIRECTION
        Translation2d robotVelVec = new Translation2d(
            robotSpeed.vxMetersPerSecond, 
            robotSpeed.vyMetersPerSecond
        );
        
        // Velocity component along the shot direction
        double velocityAlongShot = robotVelVec.getX() * Math.cos(targetAngle.getRadians()) 
                                  + robotVelVec.getY() * Math.sin(targetAngle.getRadians());
        
        // 5. GET BASE SPEEDS FROM INTERPOLATOR (stationary shot at future distance)
        TripleSpeedInterpolator.Speeds baseSpeeds = shooterInterpolator.get(futureDistance);
        
        // 6. COMPENSATE FOR MOTION
        // The main motor speed represents the note's exit velocity
        // We need to find what horizontal component it produces
        double baseHorizontalSpeed = baseSpeeds.one * Math.cos(HOOD_ANGLE_RAD);
        
        // Adjust for robot velocity along shot direction
        double requiredHorizontalSpeed = baseHorizontalSpeed - velocityAlongShot;
        
        // 7. CALCULATE SCALING FACTOR
        // Scale all motors proportionally to maintain shot characteristics
        double scaleFactor = requiredHorizontalSpeed / baseHorizontalSpeed;
        
        // 8. RETURN SCALED SPEEDS
        return new TripleSpeedInterpolator.Speeds(
            baseSpeeds.one * scaleFactor,
            baseSpeeds.two * scaleFactor,
            baseSpeeds.three * scaleFactor
        );
    }
    
    public static Rotation2d getTargetRotation(Pose2d robotPose) {
        Translation2d goalLocation = FieldConstants.GOAL_LOCATION;
        return goalLocation.minus(robotPose.getTranslation()).getAngle();
    }
    
    public static boolean isShotValid(Pose2d robotPose) {
        Translation2d goalLocation = FieldConstants.GOAL_LOCATION;
        double distance = goalLocation.getDistance(robotPose.getTranslation());
        return distance >= MIN_DISTANCE && distance <= MAX_DISTANCE;
    }
    
    public static double getDistanceToGoal(Pose2d robotPose) {
        Translation2d goalLocation = FieldConstants.GOAL_LOCATION;
        return goalLocation.getDistance(robotPose.getTranslation());
    }
}