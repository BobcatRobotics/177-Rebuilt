package frc.robot.subsystems.Shooter.Modules;

import frc.robot.subsystems.Shooter.ShooterState;

public interface ShooterModuleInterface {
    public default void configureShooterFlywheel(Configurator config)  {
    }

    public default void configureIntakeWheel(Configurator config) {
    }

    public default void configurebackspinWheelMotor(Configurator config)  {

    }

    public default void setOutput(double shooterOutput, double backspinOutput, double angleOutput) {
    }

    public default void setVelocity(ShooterState desiredState) {
    }

    public default void setVelocity(double shooterFlywheelSpeed, double shooterIntakeSpeed,
            double shooterBackspinSpeed) {

    }

    public default void setMainWheelSpeed(double shooterFlywheelSpeed) {
    }

    public default void setBackspinSpeed(double shooterBackspinSpeed) {
    }

    public default void setIntakeSpeed(double shooterIntakeSpeed) {
    }

    public default void stopMainWheel() {
    }

    public default void stopBackspinWheel() {

    }

    public default void stopIntakeWheel() {
    }

}
