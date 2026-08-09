package frc.robot.subsystems.Shooter;

public enum ShooterState {
    IDLE(0),
    SHOOT(40);

    private final double rps;

    ShooterState(double rps) {
        this.rps = rps;
    }

    public double getRPS() {
        return rps;
    }
}
