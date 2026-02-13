package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public class IntakeIOInputs {
    public double velocityRPM = 0.0;
    public double positionDeg = 0.0;
    public boolean velocityConnected = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {
  }

  public default void setVelocity(double velocity) {
  }

  public default void setPosition(IntakeState state) {
  }

  public default void runIntake(double speed){
  }

  public default void stop() {
  }
}