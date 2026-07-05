package frc.robot.subsystems.shooter;

public enum ShooterState {
    IDLE(0.0, 0.0),
    MANUAL_SPINUP(1.0, 0.0),
    MANUL_SHOOT(1.0, 0.5),

    // Uses interpolation instead of fixed values
    INTERPOLATED();

    private final double rollerSpeed;
    private final double position;
    private final boolean interpolated;

    ShooterState(double rollerSpeed, double position) {
        this.rollerSpeed = rollerSpeed;
        this.position = position;
        this.interpolated = false;
    }

    ShooterState() {
        this.rollerSpeed = 0.0;
        this.position = 0.0;
        this.interpolated = true;
    }

    public boolean isInterpolated() {
        return interpolated;
    }

    public double getPosition() {
        return position;
    }

    public double getRollerSpeed() {
        return rollerSpeed;
    }
}