package org.bobcatrobotics.Framework.StateMachine;

public interface SubsystemState {

    /**
     * Called once when the state becomes active.
     */
    default void onEnter() {}

    /**
     * Called every robot loop while this state is active.
     */
    default void execute() {}

    /**
     * Called once before leaving this state.
     */
    default void onExit() {}

    /**
     * Returns true when this state has completed.
     */
    default boolean isFinished() {
        return false;
    }
}