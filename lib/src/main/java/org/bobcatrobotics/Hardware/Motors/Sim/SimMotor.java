package org.bobcatrobotics.Hardware.Motors.Sim;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SimMotor {
    public enum ControlType{
        POSITION,VELOCITY
    }
    public enum MotorType {
        FALCON,
        FALCON_FOC,
        KRAKEN,
        KRAKEN_FOC
    }

    public default void update() {
    }

    default void setVelocity(double speedInRPS) {
    }

    default void setPosition(Rotation2d position) {
    }

    default void set(double output) {
    }

}
