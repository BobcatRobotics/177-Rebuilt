package frc.robot.subystems.Hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

  @AutoLog
  class HopperIOInputs {
    public double topVelocityRPM = 0.0;
    public double bottomVelocityRPM = 0.0;
    public boolean topMotorConnected = false;
    public boolean bottomMotorConnected = false;
  }

  default void updateInputs(HopperIOInputs inputs) {
  }

  public default void setTopVelocity(double velocity) {
  }
  public default void setBottomVelocity(double velocity) {
  }
  default void stop() {
  }
}