package frc.robot.subsystems.Carwash;

public enum CarwashState {
    IDLE(0.0),
    INTAKE(80.0),
    OUTTAKE(-10.0);

    private final double rps;

    private CarwashState(double rps){
        this.rps = rps;
    }
    public double getRPS(){
        return rps;
    }
}
