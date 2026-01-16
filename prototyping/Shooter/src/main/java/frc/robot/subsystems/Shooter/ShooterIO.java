package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;
public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double topVelocityRPM = 0.0;
    public double bottomVelocityRPM = 0.0;
    public double kickerVelocityRPM = 0.0;

    public double topAppliedVolts = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double kickerAppliedVolts = 0.0;

    public boolean topConnected = false;
    public boolean bottomConnected = false;
    public boolean kickerConnected = false;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setTopVoltage(double volts) {}
  default void setBottomVoltage(double volts) {}
  default void setKickerVoltage(double volts) {}

  default void setTopVelocity(double rpm) {}
  default void setBottomVelocity(double rpm) {}
  default void setKickerVelocity(double rpm){}

  default void stop() {}
}