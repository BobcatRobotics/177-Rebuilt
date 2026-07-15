package frc.robot.subsystems.intake;

import org.bobcatrobotics.Framework.StateMachine.SubsystemStateMachine;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperState;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    
    private final SubsystemStateMachine<IntakeState> intakeStateMachine =
            new SubsystemStateMachine<>(IntakeState.IDLE);


    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {

    intakeStateMachine.periodic();
        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Intake/inputs", inputs);
            switch (intakeStateMachine.getState()) {
            case IDLE:
                io.stop();
                break;

            case STOW:
                io.runMotors(intakeStateMachine.getState());
                break;

            case DOWN:
                io.runMotors(intakeStateMachine.getState());
                break;

            case DOWN_AND_INTAKE:
                io.runMotors(intakeStateMachine.getState());
                break;

            case DOWN_AND_OUTTAKE:
                io.runMotors(intakeStateMachine.getState());
                break;

            case OUTTAKE:
                io.runMotors(intakeStateMachine.getState());
                break;
            case INTAKE:
                io.runMotors(intakeStateMachine.getState());
                break;
        }

  }

    public void stop() {
        io.stop();
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
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
}
