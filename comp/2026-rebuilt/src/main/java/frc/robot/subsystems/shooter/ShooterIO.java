package frc.robot.subsystems.shooter;


import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double velocityOfDumperLeftUpRPS = 0;
    public double velocityOfDumperLeftDownRPS = 0;
    public double velocityOfDumperRightUpRPS = 0;
    public double velocityOfDumperRightDownRPS = 0;
    public double accelerationOfDumperLeftUp = 0;
    public double accelerationOfDumperLeftDown = 0;
    public double accelerationOfDumperRightUp = 0;
    public double accelerationOfDumperRightDown = 0;
    public double accelerationOfAdjustableHood = 0;
    public double statorCurrentOfDumperLeftUp = 0;
    public double statorCurrentOfDumperLeftDown = 0;
    public double statorCurrentOfDumperRightUp = 0;
    public double statorCurrentOfDumperRightDown = 0;
    public boolean DumperLeftUpConnected = false;
    public boolean DumperLeftDownConnected = false;
    public boolean DumperRightUpConnected = false;
    public boolean DumperRightDownConnected = false;
    public double outputOfDumperLeftUpVolts = 0;
    public double outputOfDumperLeftDownVolts = 0;
    public double outputOfDumperRightUpVolts = 0;
    public double outputOfDumperRightDownVolts = 0;
    public double outputOfAdjustableHoodVolts = 0;
    public double statorCurrentOfAdjustableHoodPositionAmps = 0; 
    public boolean adjustableHoodConnected = false; 
    public double velocityOfAdjustableHoodPositionRPS = 0;
    public double positionOfAdjustableHood = -1; 
  }

  default void updateInputs(ShooterIOInputs inputs) {

  }
  public default void stop() {
  }

  public default void setState(ShooterState state) {
  }

  public default void simulationPeriodic() {

  }

  public default void periodic() {

  }
}

