package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.Drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class AutoAimDrive extends Command {

    private final Drive drive;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    private static final double DEADBAND = 0.1;

    // Field position of hub/goal
    private final Translation2d target = new Translation2d(4.620, 4.040);

    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            0.1, // kP
            0.0,
            0.2,
            new TrapezoidProfile.Constraints(2.0, 3.0));

    public AutoAimDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    @Override
    public void execute() {

        Pose2d robotPose = drive.getPose();

        Translation2d robotTranslation = robotPose.getTranslation();

        double angleToTarget = Math.atan2(
                target.getY() - robotTranslation.getY(),
                target.getX() - robotTranslation.getX());

        double rotation = thetaController.calculate(
                robotPose.getRotation().getRadians(),
                angleToTarget);

        drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), rotation);

        Pose2d targetPose = new Pose2d(target, new Rotation2d());

        Logger.recordOutput("AutoAim/TargetPose", targetPose);

        Pose2d vectorPose = new Pose2d(
                robotPose.getTranslation(),
                new Rotation2d(
                        target.getX() - robotPose.getX(),
                        target.getY() - robotPose.getY()));

        Logger.recordOutput("AutoAim/RobotToTargetVector", vectorPose);

        Pose2d aimPose = new Pose2d(
                robotPose.getTranslation(),
                new Rotation2d(angleToTarget));

        Logger.recordOutput("AutoAim/AimPose", aimPose);

        double distance = robotPose.getTranslation().getDistance(target);

        Logger.recordOutput("AutoAim/DistanceToTarget", distance);

        Logger.recordOutput("AutoAim/ThetaError", thetaController.getPositionError());
        Logger.recordOutput("AutoAim/ThetaSetpoint", thetaController.getSetpoint().position);
        Logger.recordOutput("AutoAim/ThetaVelocitySetpoint", thetaController.getSetpoint().velocity);

        Translation2d[] shotLine = new Translation2d[] {
                robotPose.getTranslation(),
                target
        };

        Logger.recordOutput("AutoAim/ShotLine", shotLine);
    }

    @Override
    public void end(boolean interrupted) {
        drive(0, 0, 0);
    }

    private void drive(double x, double y, double rotation) {

        // Get linear velocity
        Translation2d linearVelocity = getLinearVelocityFromJoysticks(x, y);

        // Apply rotation deadband
        double omega = MathUtil.applyDeadband(rotation, DEADBAND);

        // Square rotation value for more precise control
        omega = Math.copySign(omega * omega, omega);

        // Convert to field relative speeds & send command
        ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                omega * drive.getMaxAngularSpeedRadPerSec()*0.1);
        boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
                isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
    }

    private Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();
    }

}
