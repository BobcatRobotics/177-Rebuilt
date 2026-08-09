package frc.robot.subsystems.Intake;

public enum IntakeState {
    IDLE(0),
    INTAKE(40),
    EJECT(40);      //our OUTTAKE?

    private final double rps;

    IntakeState(double rps) {
        this.rps = rps;
    }

    public double getRPS() {
        return rps;
    }

}
