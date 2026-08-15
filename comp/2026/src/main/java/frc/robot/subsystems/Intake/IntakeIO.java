package frc.robot.subsystems.Intake;

public interface IntakeIO {

    // Rollers
    void setRollerVelocity(double rps);
    void stopRollers();
    double getLeftRollerVelocityRPS();
    double getRightRollerVelocityRPS();
    double getRollerAverageVelocityRPS();


    // Pivot
    void setPivotAngle(double angleDegrees);
    void stopPivot();
    double getPivotAngleDegrees();
    boolean isPivotAtTarget(double targetAngleDegrees);


    // Simulation
    default void simulationPeriodic() {
        // Nothing to do on real robot
    }
}