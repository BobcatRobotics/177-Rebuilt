package frc.robot.subsystems.Hood;

public interface HoodIO {

    void setAngle(double angleDegrees);

    void stop();

    double getAngleDegrees();

    boolean atTargetAngle(double targetAngleDegrees);

    double getHoodMotorVelocityRPS();

    default void simulationPeriodic() {
        // Nothing to do on the real robot
    }
}