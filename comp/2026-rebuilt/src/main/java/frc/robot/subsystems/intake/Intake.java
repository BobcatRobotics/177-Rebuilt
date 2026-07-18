package frc.robot.subsystems.intake;

import org.bobcatrobotics.Framework.StateMachine.SubsystemStateMachine;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private SubsystemStateMachine<IntakeState> intakeStateMachine = new SubsystemStateMachine<IntakeState>(
            IntakeState.IDLE);

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {

        intakeStateMachine.periodic();
        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Intake/inputs", inputs);
        IntakeState currentState = intakeStateMachine.getState();
        switch (currentState) {
            case IDLE:
                io.stop();
                break;

            case STOW:
                io.setPosition(intakeStateMachine.getState());
                break;

            case DOWN:
                io.setPosition(intakeStateMachine.getState());
                break;

            case DOWN_AND_INTAKE:
                io.runMotors(intakeStateMachine.getState());
                break;

            case DOWN_AND_OUTTAKE:
                io.runMotors(intakeStateMachine.getState());
                break;

            case OUTTAKE:
                io.setVelocity(intakeStateMachine.getState());
                break;
            case INTAKE:
                io.setVelocity(intakeStateMachine.getState());
                break;
        }
    }

    public void stop() {
        io.stop();
    }

    public void simulationPeriodic() {

        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Intake/inputs", inputs);

        IntakeState currentState = intakeStateMachine.getState();
        switch (currentState) {
            case IDLE:
                io.stop();
                break;

            case STOW:
                io.setPosition(intakeStateMachine.getState());
                break;

            case DOWN:
                io.setPosition(intakeStateMachine.getState());
                break;

            case DOWN_AND_INTAKE:
                io.runMotors(intakeStateMachine.getState());
                break;

            case DOWN_AND_OUTTAKE:
                io.runMotors(intakeStateMachine.getState());
                break;

            case OUTTAKE:
                io.setVelocity(intakeStateMachine.getState());
                break;
            case INTAKE:
                io.setVelocity(intakeStateMachine.getState());
                break;
        }

    }

    public void setState(IntakeState state) {
        intakeStateMachine.setState(state);
    }

    public IntakeState getState() {
        return intakeStateMachine.getState();
    }

    public boolean inState(IntakeState state) {
        return intakeStateMachine.isInState(state);
    }

    public boolean isAtPosition(IntakeState state) {
        return io.isAtPosition(state);
    }

    public boolean isAtSpeed(IntakeState state) {
        return io.isAtSpeed(state);
    }
}
