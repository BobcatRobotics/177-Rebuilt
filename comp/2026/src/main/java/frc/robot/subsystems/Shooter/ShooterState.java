package frc.robot.subsystems.Shooter;

public enum ShooterState {
    IDLE(0.0),
    SHOOT(0.0),
    REVERSE_SHOOT(0.0),
    SPIN_UP(0.0);
    
    private final double rps;
    ShooterState(double rps){
        this.rps = rps;
    }
    
    public double getRPS() {
    return rps;
  }
}
