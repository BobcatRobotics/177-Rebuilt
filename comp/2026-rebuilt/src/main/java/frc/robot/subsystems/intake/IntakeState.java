package frc.robot.subsystems.intake;

public enum IntakeState {
    IDLE(0.0),
    STOW(0.0),
    DOWN(1.0),
    DOWN_AND_INTAKE(1.0, 1.0),
    DOWN_AND_OUTTAKE(1.0, -1.0),
    OUTTAKE(-1.0, true),
    INTAKE(1.0, true);

    private final double position;
    private final double rollerSpeed;

    IntakeState(double position) {
        this(position, 0.0);
    }

    IntakeState(double value, boolean rollerOnly) {
        this(0.0, value);
    }

    IntakeState(double position, double rollerSpeed) {
        this.position = position;
        this.rollerSpeed = rollerSpeed;
    }

    public double getPosition() {
        return position;
    }

    public double getRollerSpeed() {
        return rollerSpeed;
    }
}