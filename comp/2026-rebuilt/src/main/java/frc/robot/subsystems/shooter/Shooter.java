package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private ShooterState currentState = ShooterState.IDLE;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/inputs", inputs);

    }

    public Translation2d[] getShotLine(double distance) {
        Pose2d robotPose = RobotState.getInstance().robotPose;
        Pose2d newPose = robotPose.transformBy(new Transform2d(
                new Translation2d(Units.inchesToMeters(distance), 0),
                new Rotation2d()));
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d targetTranslation = newPose.getTranslation();
        Translation2d[] shotLine = new Translation2d[] {
                robotTranslation,
                targetTranslation
        };
        return shotLine;
    }

    public Translation2d[] getTargetShotLine(Translation2d targetTranslation) {
        Pose2d robotPose = RobotState.getInstance().robotPose;
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d[] shotLine = new Translation2d[] {
                robotTranslation,
                targetTranslation
        };
        return shotLine;
    }

    public void stop() {
        io.stop();
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public void setState(ShooterState state) {
        currentState = state;
        io.setState(currentState);
    }

    public ShooterState getState() {
        return currentState;
    }

    public boolean atSpeed() {
        boolean isAtTolerance = false;
        isAtTolerance = io.atSpeed(currentState.getRollerSpeed());
        return isAtTolerance;
    }
}