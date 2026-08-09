package frc.robot.subsystems.Hopper;

public interface HopperIO {
    void setVelocity(double rps);

    void stop();

    double getVelocityRPS();
}
