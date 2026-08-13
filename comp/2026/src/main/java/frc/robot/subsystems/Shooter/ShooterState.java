package frc.robot.subsystems.Shooter;

public enum ShooterState {
    IDLE(0);
    
    
    private final double rps;
    public ShooterState(rps){
        this.rps = rps;
    }
}
