package frc.robot.subsystems.Carwash;

public enum CarwashState {
    IDLE(0.0),
    FEED(80.0),
    REVERSE(-80.0),
    SPIN_UP(-13.0),
    TARGETING(80.0),
    INTERPOLATING(80.0);

    private final double rps;

    CarwashState(double rps) {
        this.rps = rps;
    }

    public double getRPS() {
        return rps;
    }
}
