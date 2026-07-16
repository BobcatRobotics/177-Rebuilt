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
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;

public class PassingShot extends Command {
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
    private Translation2d hubTarget;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;

    public PassingShot(Drive drive) {

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

    public PassingShot(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

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

    public PassingShot(Drive drive, double velocity, double acceleration) {

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
        hubTarget = HubUtil.getMyHubCoordinates(RobotState.getInstance().alliance).toPose2d().getTranslation();
        Pose2d robotPose = drive.getPose();
        if (robotPose.getY() > hubTarget.getY()) {
            target = HubUtil.getDepoPassingCoordiante(RobotState.getInstance().alliance).toPose2d().getTranslation();
            RobotState.getInstance().targettedDistance = RobotState.getInstance().depoDistance;
        } else {

            target = HubUtil.getOutpostPassingCoordinate(RobotState.getInstance().alliance).toPose2d().getTranslation();
            RobotState.getInstance().targettedDistance = RobotState.getInstance().outpostDistance;

        }
        Rotation2d targetHeading = new Rotation2d(target.getX() - robotPose.getX(), target.getY() - robotPose.getY());
        boolean isAtSetpoint = isAligned(robotPose.getRotation().getDegrees(), targetHeading.getDegrees());
        RobotState.getInstance().isRobotAlignedToHub = isAtSetpoint;
        Logger.recordOutput("AlignPassing/RobotHeadingPose", robotPose);
        Logger.recordOutput("AlignPassing/TargetHeadingAngle", new Pose2d(robotPose.getTranslation(), targetHeading));
        Logger.recordOutput("AlignPassing/IsAligned", isAtSetpoint);
        RobotState.getInstance().isRobotAlignedToPassingLoc = isAtSetpoint;
        drive(targetHeading.getRadians());

    }

    public boolean isAligned() {
        Translation2d targetTranslation = new Translation2d();
        Pose2d robotPose = drive.getPose();
        if (robotPose.getY() > target.getY()) {

            targetTranslation = HubUtil.getDepoPassingCoordiante(RobotState.getInstance().alliance).toPose2d()
                    .getTranslation();
        } else {
            targetTranslation = HubUtil.getOutpostPassingCoordinate(RobotState.getInstance().alliance).toPose2d()
                    .getTranslation();
        }
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
        // Logger.recordOutput("Chassis Velocity - x", vx);
        // Logger.recordOutput("Chassis Velocity - y", vy);

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
        } else {
            Translation2d linearVelocity = DriveCommands.getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                    ySupplier.getAsDouble());
            // Convert to field relative speeds & send command
            speeds = new ChassisSpeeds(
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
