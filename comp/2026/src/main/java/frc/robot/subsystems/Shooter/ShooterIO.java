package frc.robot.subsystems.Shooter;

public interface ShooterIO {
    void setFlywheelVelocity(double rps);

    void stopFlywheel();

    // Average velocity of all 4 flywheels
    double getFlywheelVelocityRPS();

    double getLeftInnerVelocityRPS();

    double getRightInnerVelocityRPS();

    double getLeftOuterVelocityRPS();

    double getRightOuterVelocityRPS();

    boolean atTargetSpeed(double targetRPS);

    default void simulationPeriodic() {
        // Nothing to do on real robot
    }
}