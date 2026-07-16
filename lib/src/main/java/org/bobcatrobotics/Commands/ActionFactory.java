package org.bobcatrobotics.Commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Factory class for creating WPILib {@link Command} objects with standardized behaviors and logging.
 *
 * <p>
 * Provides utilities for generating single-execution actions, continuous actions, applying timeouts,
 * conditional termination, and integrating AdvantageKit logging for action states.
 * </p>
 */
public class ActionFactory {
    /** Flag indicating whether a timeout has been applied to the last command. */
    public boolean hasTimeout = false;

    /** Flag indicating whether a condition has been applied to the last command. */
    public boolean hasCondition = false;

    /**
     * Creates a command that executes a runnable once.
     *
     * <p>
     * Uses WPILib's {@link InstantCommand}. This command does not require subsystems and can run
     * in parallel with other commands that use the same subsystems.
     * </p>
     *
     * @param commandName the name of the command for debugging/logging
     * @param runBehaviour the action to run
     * @return a {@link Command} that executes once
     */
    public Command singleAction(String commandName, Runnable runBehaviour) {
        return new InstantCommand(runBehaviour).withName(commandName);
    }

    /**
     * Creates a command that executes a runnable once, requiring the given subsystems.
     *
     * <p>
     * Uses {@link Commands#runOnce(Runnable, Subsystem...)}. This command ensures that if another
     * command requiring the same subsystems is scheduled, it will interrupt this one.
     * </p>
     *
     * @param commandName the name of the command
     * @param runBehaviour the action to run
     * @param subsystems the subsystems required by this command
     * @return a {@link Command} that executes once and requires subsystems
     */
    public Command singleAction(String commandName, Runnable runBehaviour,
            Subsystem... subsystems) {
        return Commands.runOnce(runBehaviour, subsystems).withName(commandName);
    }

    /**
     * Creates a continuous command that repeatedly executes a runnable until interrupted.
     *
     * <p>
     * Uses WPILib's {@link RunCommand}. This version does not require any subsystems. Defines
     * behavior for when the command is interrupted.
     * </p>
     *
     * @param commandName the name of the command
     * @param runBehaviour the action to run continuously
     * @param onInterrupt action to perform when the command is interrupted
     * @return a {@link Command} that continuously runs until interrupted
     */
    public Command continuousAction(String commandName, Runnable runBehaviour,
            Runnable onInterrupt) {
        return new RunCommand(runBehaviour).withName(commandName).handleInterrupt(onInterrupt);
    }

    /**
     * Creates a continuous command that repeatedly executes a runnable until interrupted,
     * requiring the specified subsystems.
     *
     * @param commandName the name of the command
     * @param runBehaviour the action to run continuously
     * @param onInterrupt action to perform when the command is interrupted
     * @param subsystems the subsystems required by this command
     * @return a {@link Command} that continuously runs and requires subsystems
     */
    public Command continuousAction(String commandName, Runnable runBehaviour, Runnable onInterrupt,
            Subsystem... subsystems) {
        return new RunCommand(runBehaviour, subsystems).withName(commandName)
                .handleInterrupt(onInterrupt);
    }

    /**
     * Adds a timeout to the given command.
     *
     * @param action the command to apply the timeout to
     * @param time the timeout in seconds
     * @return the same command with a timeout applied
     */
    public Command addTimeout(Command action, double time) {
        hasTimeout = true;
        return action.withTimeout(time);
    }

    /**
     * Adds a conditional termination to the given command.
     *
     * @param action the command to apply the condition to
     * @param condition a {@link BooleanSupplier} which, when true, cancels the command
     * @return the same command with a conditional termination applied
     */
    public Command addCondition(Command action, BooleanSupplier condition) {
        hasCondition = true;
        return action.until(condition);
    }

    /**
     * Wraps a command with logging using AdvantageKit, recording initialization, execution,
     * and completion states.
     *
     * @param action the command to log
     * @param keyName the name of the key in AdvantageKit to store the action state
     * @return the original command wrapped with logging
     */
    public Command logCommand(Command action, String keyName){
        Command logInitialization = singleAction(action.getName(), ()-> {
            Logger.recordOutput(action.getSubsystem() + "/" + keyName, ActionState.Initialized);
        });
        Command logExecuting = singleAction(action.getName(), ()-> {
            Logger.recordOutput(action.getSubsystem() + "/" + keyName, ActionState.Executing);
        });
        Command logCommand = logExecuting.beforeStarting(logInitialization).finallyDo(()-> {
            Logger.recordOutput(action.getSubsystem() + "/" + keyName, ActionState.Finished);
        });
        action.alongWith(logCommand);
        return action;
    }
}
