package frc.robot.subsystems.Hopper;

public interface HopperIO {
    void setVelocity(double rps);

    void stop();

    double getVelocityRPS();

    default void simulationPeriodic(){
        // Nothing to do on real robot
    }
}
