package frc.robot.subsystems.Shooter;

public enum ShooterState {
    OFF(0.0),
    IDLE(0.0),
    SHOOT(60.0),
    REVERSE(-20.0);

    private final double flywheelRPS;

    ShooterState(double flywheelRPS) {
        this.flywheelRPS = flywheelRPS;
    }

    public double getFlywheelRPS() {
        return flywheelRPS;
    }
}