package frc.robot.subsystems.carwash;

public enum CarwashState {
    IDLE(0.0), 
    FEED(80.0),
    OUTAKE(-20.0);

    private final double carwashSpeed;

    CarwashState(double velocity) {
        this.carwashSpeed = velocity;
    }

    public double getCarwashSpeed() {
        return carwashSpeed;
    }
}
