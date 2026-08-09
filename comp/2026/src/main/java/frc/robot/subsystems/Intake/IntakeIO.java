package frc.robot.subsystems.Intake;

public interface IntakeIO {
        void setVelocity(double rps);

    void stop();

    double getVelocityRPS();
}
