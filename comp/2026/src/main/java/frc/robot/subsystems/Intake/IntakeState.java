package frc.robot.subsystems.Intake;

public enum IntakeState {

    IDLE(
        0.0,     // roller RPS
        0.0      // pivot angle
    ),

    DEPLOYED(
        0.0,
        60.0
    ),

    INTAKE(
        40.0,
        60.0
    ),

    EJECT(
        -40.0,
        60.0
    );

    private final double rollerRPS;
    private final double pivotAngleDegrees;

    IntakeState(
            double rollerRPS,
            double pivotAngleDegrees) {

        this.rollerRPS = rollerRPS;
        this.pivotAngleDegrees = pivotAngleDegrees;
    }

    public double getRollerRPS() {
        return rollerRPS;
    }

    public double getPivotAngleDegrees() {
        return pivotAngleDegrees;
    }
}