package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.bobcatrobotics.GameSpecific.Rebuilt.HubUtil;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;

public class AlignToHub extends Command {
    private boolean done = false;
    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 7.5;
    private static final double ANGLE_KD = 0.4;
    // private static final double ANGLE_MAX_VELOCITY = 8.0;
    // private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private final double ANGLE_MAX_VELOCITY;
    private final double ANGLE_MAX_ACCELERATION;

    private final Drive drive;
    private final ProfiledPIDController angleController;
    // Field position of hub/goal
    private Translation2d target;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;

    public AlignToHub(Drive drive) {

        ANGLE_MAX_VELOCITY = 8.0;
        ANGLE_MAX_ACCELERATION = 20.0;
        this.drive = drive;

            // Create PID controller
            angleController = new ProfiledPIDController(
                    ANGLE_KP,
                    0.0,
                    ANGLE_KD,
                    new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
            angleController.enableContinuousInput(-Math.PI, Math.PI);
            angleController.reset(drive.getRotation().getRadians());
    }

        public AlignToHub(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

        ANGLE_MAX_VELOCITY = 8.0;
        ANGLE_MAX_ACCELERATION = 20.0;
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        // Create PID controller
        angleController = new ProfiledPIDController(
                ANGLE_KP,
                0.0,
                ANGLE_KD,
                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.reset(drive.getRotation().getRadians());
        
    }

        public AlignToHub(Drive drive, double velocity, double acceleration) {

            ANGLE_MAX_VELOCITY = velocity;
            ANGLE_MAX_ACCELERATION = acceleration;
            this.drive = drive;

            // Create PID controller
            angleController = new ProfiledPIDController(
                    ANGLE_KP,
                    0.0,
                    ANGLE_KD,
                    new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
            angleController.enableContinuousInput(-Math.PI, Math.PI);
            angleController.reset(drive.getRotation().getRadians());
        }




    @Override
    public void execute() {
            target = HubUtil.getMyHubCoordinates(RobotState.getInstance().alliance).toPose2d().getTranslation();
            Pose2d robotPose = drive.getPose();
            Rotation2d targetHeading = new Rotation2d(target.getX() - robotPose.getX(), target.getY() - robotPose.getY());
            boolean isAtSetpoint = isAligned(robotPose.getRotation().getDegrees(), targetHeading.getDegrees());
            RobotState.getInstance().isRobotAlignedToHub = isAtSetpoint;
            Logger.recordOutput("Align/RobotHeadingPose", robotPose);
            Logger.recordOutput("Align/TargetHeadingAngle", new Pose2d(robotPose.getTranslation(), targetHeading));
            Logger.recordOutput("Align/IsAligned", isAtSetpoint);
            drive(targetHeading.getRadians());
            
           
    }

      

    public boolean isAligned() {
        Translation2d targetTranslation = HubUtil.getMyHubCoordinates(RobotState.getInstance().alliance).toPose2d()
                .getTranslation();
        Pose2d robotPose = drive.getPose();
        Rotation2d targetHeading = new Rotation2d(targetTranslation.getX() - robotPose.getX(),
                targetTranslation.getY() - robotPose.getY());
        return isAligned(robotPose.getRotation().getDegrees(), targetHeading.getDegrees());
    }

    public boolean isAligned(double actual, double setpoint) {
        boolean isAtTolerance = false;
        boolean isMainFlywheelWithinTolerance = false;

        double MAIN_SPEED_TOLERANCE = 1;
        isMainFlywheelWithinTolerance = Math.abs(actual - setpoint) <= MAIN_SPEED_TOLERANCE;
        if (isMainFlywheelWithinTolerance) {
            isAtTolerance = true;
        }
        return isAtTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopWithX();
    }

    public void drive(double chassisHeadingInRadians) {
        // Convert to field relative speeds & send command
        ChassisSpeeds currentSpeeds = drive.getChassisSpeeds();

        // Calculate angular speed
        double omega = angleController.calculate(
                drive.getRotation().getRadians(), chassisHeadingInRadians);
        // double vx = RobotState.getInstance().vx;
        // double vy = RobotState.getInstance().vy;
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, omega);

        Logger.recordOutput("Align/ThetaError", angleController.getPositionError());
        Logger.recordOutput("Align/ThetaSetpoint", angleController.getSetpoint().position);
        Logger.recordOutput("Align/ThetaVelocitySetpoint", angleController.getSetpoint().velocity);

        if (xSupplier == null && ySupplier == null) {
            // Convert to field relative speeds & send command
            boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        isFlipped
                                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                : drive.getRotation()));
        }
        else {
            Translation2d linearVelocity =
                    DriveCommands.getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
               // Convert to field relative speeds & send command
             speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
            boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        isFlipped
                                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                : drive.getRotation()));
        }
    }
}

