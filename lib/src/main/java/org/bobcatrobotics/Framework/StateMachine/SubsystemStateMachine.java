package org.bobcatrobotics.Framework.StateMachine;

public class SubsystemStateMachine<T extends Enum<T> & SubsystemState> {

    private T currentState;
    private T previousState;

    public SubsystemStateMachine(T initialState) {
        currentState = initialState;
        currentState.onEnter();
    }

    public void setState(T newState) {

        if (newState == currentState) {
            return;
        }

        currentState.onExit();

        previousState = currentState;
        currentState = newState;

        currentState.onEnter();
    }

    public void periodic() {
        currentState.execute();
    }

    public boolean isFinished() {
        return currentState.isFinished();
    }

    public boolean isInState(T state) {
        return currentState == state;
    }

    public T getState() {
        return currentState;
    }

    public T getPreviousState() {
        return previousState;
    }
}