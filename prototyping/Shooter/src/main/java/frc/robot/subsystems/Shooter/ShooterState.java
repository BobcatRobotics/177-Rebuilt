package frc.robot.subsystems.Shooter;

public enum ShooterState {
  IDLE(0.0, 0.0, 0.0),
  HUB(1800.0, 1200.0, 0),
  MID(5200.0, 4800.0, 0),
  FAR(6000.0, 5600.0, 0);

  public final double topRPM;
  public final double bottomRPM;
  public final double kickerRPM;

  ShooterState(double topRPM, double bottomRPM, double kickerRPM) {
    this.topRPM = topRPM;
    this.bottomRPM = bottomRPM;
    this.kickerRPM = kickerRPM;
  }
}