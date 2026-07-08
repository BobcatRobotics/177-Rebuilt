package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private IntakeState currentState = IntakeState.IDLE;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Intake/inputs", inputs);
    }

    public void stop() {
        io.stop();
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public void setState(IntakeState state) {
        currentState = state;
        io.setState(currentState);
    }

    public IntakeState getState() {
        return currentState;
    }
}
