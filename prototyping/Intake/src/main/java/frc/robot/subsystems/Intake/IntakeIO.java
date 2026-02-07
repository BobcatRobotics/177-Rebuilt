package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  class IntakeIOInputs {
    public double velocityRPM = 0.0;
    public double positionDeg = 0.0;
    public boolean velocityConnected = false;
  }

  default void updateInputs(IntakeIOInputs inputs) {
  }

  public default void setVelocity(double velocity) {
  }

  public void runIntake(double speed){

  }

  default void stop() {
  }
}