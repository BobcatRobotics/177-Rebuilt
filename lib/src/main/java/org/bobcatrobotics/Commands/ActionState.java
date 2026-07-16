package org.bobcatrobotics.Commands;

/**
 * Represents the possible states of an action or command within the robot code.
 *
 * <p>
 * This enum can be used to track the lifecycle of a command or action, from initialization to
 * completion or interruption.
 * </p>
 */
public enum ActionState {
    /** The action state is unknown or uninitialized. */
    Unknown,

    /** The action has been initialized but has not started executing yet. */
    Initialized,

    /** The action is currently executing. */
    Executing,

    /** The action has finished successfully. */
    Finished,

    /** The action was interrupted before completion. */
    Interrupted
}
