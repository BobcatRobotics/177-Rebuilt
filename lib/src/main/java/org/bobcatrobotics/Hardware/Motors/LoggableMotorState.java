package org.bobcatrobotics.Hardware.Motors;
import org.littletonrobotics.junction.Logger;

/**
 * Holds and logs an enum-based state using AdvantageKit.
 */
public class LoggableMotorState {
    public enum MotorState {
        UNKNOWN,IDLE,REVERSE,FORWARD,
    }
    private MotorState currentState;
    private final String logPath;

    /**
     * @param logPath AdvantageKit output key (e.g. "Shooter/State")
     * @param initialState Initial enum value
     */
    public LoggableMotorState(String logPath, MotorState initialState) {
        this.logPath = logPath;
        set(initialState); // logs immediately
    }

    /**
     * Sets the state and logs it if changed.
     */
    public void set(MotorState newState) {
        if (newState != currentState) {
            currentState = newState;
            Logger.recordOutput(logPath,newState);
        }
    }

    /**
     * @return Current state
     */
    public MotorState get() {
        return currentState;
    }

    /**
     * @return true if current state equals provided state
     */
    public boolean is(MotorState state) {
        return currentState == state;
    }
}
