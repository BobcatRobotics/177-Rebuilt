package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterTable;

public class ShootOnTheMove{
    private static final double HOOD_ANGLE_RAD = Math.toRadians(45.0); // Need to get
    private final Drive drive;
    private final Shooter shooter;

    public ShootOnTheMove(Drive drive, Shooter shooter) {
        this.drive = drive;
        this.shooter = shooter;
    }

    public static double calculateRequiredVelocity(Pose2d robotPose, ChassisSpeeds robotSpeed) {
        // 1. LATENCY COMP
        double latency = 0.15; // Need to tune
        Translation2d futurePos = robotPose.getTranslation().plus(
            new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
        );

        // 2. GET TARGET VECTOR
        Translation2d goalLocation = FieldConstants.GOAL_LOCATION; // Need to add
        Translation2d targetVec = goalLocation.minus(futurePos);
        double dist = targetVec.getNorm();
        
        // 3. ROTATE ROBOT VELOCITY INTO SHOOTER'S FRAME
        // Get the angle the robot needs to face
        Rotation2d targetAngle = targetVec.getAngle();
        
        // Robot velocity in field frame
        Translation2d robotVelVec = new Translation2d(
            robotSpeed.vxMetersPerSecond, 
            robotSpeed.vyMetersPerSecond
        );
        
        // Project robot velocity onto the shooting direction
        // Only the component along the target vector matters for RPM compensation
        double velocityAlongShot = robotVelVec.getX() * Math.cos(targetAngle.getRadians()) 
                                  + robotVelVec.getY() * Math.sin(targetAngle.getRadians());

        double idealHorizontalSpeed = ShooterTable.getSpeed(dist); //Need to make table

        // 5. COMPENSATE FOR MOTION
        // Add the velocity component (positive if moving toward goal, negative if away)
        double requiredHorizontalSpeed = idealHorizontalSpeed - velocityAlongShot;

        // 6. SOLVE FOR RPM WITH FIXED HOOD ANGLE
        double totalExitVelocity = requiredHorizontalSpeed / Math.cos(HOOD_ANGLE_RAD);
        
        return totalExitVelocity;
    }
    
    public static Rotation2d getTargetRotation(Pose2d robotPose) {
        Translation2d goalLocation = FieldConstants.GOAL_LOCATION;
        return goalLocation.minus(robotPose.getTranslation()).getAngle();
    }

}