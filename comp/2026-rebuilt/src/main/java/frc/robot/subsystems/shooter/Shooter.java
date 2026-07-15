package frc.robot.subsystems.shooter;

import org.bobcatrobotics.Framework.StateMachine.SubsystemStateMachine;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.IntakeState;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final SubsystemStateMachine<ShooterState> shooterStateMachine =
            new SubsystemStateMachine<>(ShooterState.IDLE);

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/inputs", inputs);
        switch (shooterStateMachine.getState()) {
            case IDLE:
                io.stop();
                break;

            case MANUAL_SPINUP:
                io.runMotors(shooterStateMachine.getState());
                break;

            case MANUL_SHOOT:
                io.runMotors(shooterStateMachine.getState());
                break;

            case INTERPOLATED:
                io.runMotors(shooterStateMachine.getState());
                break;
        }

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
        shooterStateMachine.setState(state);
    }

    public ShooterState getState() {
        return shooterStateMachine.getState();
    }

    public boolean atSpeed() {
        boolean isAtTolerance = false;
        isAtTolerance = io.atSpeed(shooterStateMachine.getState().getRollerSpeed());
        return isAtTolerance;
    }

        public boolean inState(ShooterState state) {
        return shooterStateMachine.isInState(state);
    }
}