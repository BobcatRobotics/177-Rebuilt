package frc.robot.subsystems.carwash;

import org.bobcatrobotics.Framework.StateMachine.SubsystemStateMachine;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Carwash extends SubsystemBase {

    private final CarwashIO io;
    private final CarwashIOInputsAutoLogged inputs = new CarwashIOInputsAutoLogged();

    private final SubsystemStateMachine<CarwashState> carwashStateMachine =
            new SubsystemStateMachine<>(CarwashState.IDLE);

    public Carwash(CarwashIO io) {
        this.io = io;

    }

    @Override
    public void periodic() {
        carwashStateMachine.periodic();

        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Carwash/inputs", inputs);


        switch (carwashStateMachine.getState()) {
            case IDLE:
                io.stop();
                break;

            case FEED:
                io.runMotors(carwashStateMachine.getState());
                break;

            case OUTTAKE:
                io.runMotors(carwashStateMachine.getState());
                break;
        }

    }

    public void stop() {
        io.stop();
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public void setState(CarwashState state) {
        carwashStateMachine.setState(state);
    }

    public CarwashState getState() {
        return carwashStateMachine.getState();
    }

    public boolean inState(CarwashState state) {
        return carwashStateMachine.isInState(state);
    }
}
