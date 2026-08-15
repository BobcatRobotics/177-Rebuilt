package frc.robot.subsystems.Hood;

public enum HoodState {
    IDLE(0.0),
    SPEAKER(35.0),
    AMP(60.0);

    private final double angleDegrees;

    HoodState(double angleDegrees) {
        this.angleDegrees = angleDegrees;
    }

    public double getAngleDegrees() {
        return angleDegrees;
    }
}
