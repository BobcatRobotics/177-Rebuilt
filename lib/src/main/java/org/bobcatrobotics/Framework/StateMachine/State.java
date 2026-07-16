package org.bobcatrobotics.Framework.StateMachine;

public interface State<T> {

    default void initialize(T subsystem) {}

    default void execute(T subsystem) {}

    default void end(T subsystem) {}
}