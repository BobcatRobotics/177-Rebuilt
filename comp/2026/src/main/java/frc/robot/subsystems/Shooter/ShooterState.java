package frc.robot.subsystems.Shooter;

public enum ShooterState {
    IDLE(0.0, 30),
    SHOOT(0.0, 30),
    REVERSE_SHOOT(0.0, 30),
    SPIN_UP(0.0, 30);
    
    private final double rps;
    private final double pos;
    ShooterState(double rps, double hoodAngle){
        this.rps = rps;
        this.pos = hoodAngle;
    }
    
    public double getRPS() {
    return rps;
  } 
  public double getHoodAngle() {
    return pos;
  }     
}
