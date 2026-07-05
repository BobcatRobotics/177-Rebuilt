package org.bobcatrobotics.Framework.StateMachine;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class StateMachine<T, S extends Enum<S> & State<T>> {

    private final T subsystem;

    private final Timer timer = new Timer();

    private final S defaultState;

    private S currentState;
    private S previousState;

    private final String logKey;

    public StateMachine(
            T subsystem,
            S defaultState,
            String logKey) {

        this.subsystem = subsystem;
        this.defaultState = defaultState;
        this.logKey = logKey;

        currentState = defaultState;
        previousState = defaultState;

        timer.start();

        currentState.initialize(subsystem);
    }

    public void periodic() {

        Logger.recordOutput(logKey + "/CurrentState", currentState.name());
        Logger.recordOutput(logKey + "/PreviousState", previousState.name());
        Logger.recordOutput(logKey + "/TimeInState", timer.get());

        currentState.execute(subsystem);
    }

    public void setState(S newState) {

        if (newState == currentState) {
            return;
        }

        currentState.end(subsystem);

        previousState = currentState;
        currentState = newState;

        timer.restart();

        currentState.initialize(subsystem);

        Logger.recordOutput(
                logKey + "/Transition",
                previousState.name() + " -> " + currentState.name());
    }

    public void reset() {
        setState(defaultState);
    }

    public S getState() {
        return currentState;
    }

    public S getPreviousState() {
        return previousState;
    }

    public S getDefaultState() {
        return defaultState;
    }

    public double timeInState() {
        return timer.get();
    }

    public boolean isState(S state) {
        return currentState == state;
    }

    public boolean stateChanged() {
        return currentState != previousState;
    }
}