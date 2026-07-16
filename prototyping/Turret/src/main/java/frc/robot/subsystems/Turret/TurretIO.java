package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

  @AutoLog
  class TurretIOInputs {
    public double velocityRPM = 0.0;
    public double positionDeg = 0.0;
    public boolean positionConnected = false;
  }

  default void updateInputs(TurretIOInputs inputs) {
  }

  public default void setPosition(double position) {
  }

  public default void holdPosition() {

  }

  default void stop() {
  }
}