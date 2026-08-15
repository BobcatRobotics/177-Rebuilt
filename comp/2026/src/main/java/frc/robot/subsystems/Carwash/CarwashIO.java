package frc.robot.subsystems.Carwash;

public interface CarwashIO {

    void setVelocity(double rps);

    void stop();

    double getVelocityRPS();

    boolean atTargetSpeed(double targetRPS);

    default void simulationPeriodic() {
        // Nothing to do on the real robot
    }
}