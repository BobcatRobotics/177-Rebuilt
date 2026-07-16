package frc.robot.subsystems.carwash;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Carwash extends SubsystemBase {

    private final CarwashIO io;
    private final CarwashIOInputsAutoLogged inputs = new CarwashIOInputsAutoLogged();

    private CarwashState currentState = CarwashState.IDLE;

    public Carwash(CarwashIO io) {
        this.io = io;

    }

    @Override
    public void periodic() {
        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Carwash/inputs", inputs);

    }

    public void stop() {
        io.stop();
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public void setState(CarwashState state) {
        currentState = state;
        io.setState(currentState);
    }

    public CarwashState getState() {
        return currentState;
    }
}
