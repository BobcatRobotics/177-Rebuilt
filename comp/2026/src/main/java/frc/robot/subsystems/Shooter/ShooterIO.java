package frc.robot.subsystems.Shooter;

public interface ShooterIO {

    void setVelocity(double rps);

    void stop();

    double getVelocityRPS();
}
