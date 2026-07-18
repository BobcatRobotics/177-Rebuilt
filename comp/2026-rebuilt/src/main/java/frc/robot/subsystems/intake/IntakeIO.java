package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.hopper.HopperState;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public double velocityOfIntakePositionRPS = 0.0;
    public double statorCurrentOfIntakePositionAmps = 0.0;
    public double outputOfIntakePositionVolts = 0.0;
    public double accelerationOfIntakePosition = 0.0;
    public boolean positionConnected = false;
    public double intakePosition = -1;

    public double leftVelocityOfIntakeSpeedRPS = 0.0;
    public double leftStatorCurrentOfIntakeSpeedAmps = 0.0;
    public double leftOutputOfIntakeSpeedVolts = 0.0;
    public double leftAccelerationOfIntakeSpeed = 0.0;
    public boolean leftVelocityMotorConnected = false;

    public double rightVelocityOfIntakeSpeedRPS = 0.0;
    public double rightStatorCurrentOfIntakeSpeedAmps = 0.0;
    public double rightOutputOfIntakeSpeedVolts = 0.0;
    public double rightAccelerationOfIntakeSpeed = 0.0;
    public boolean rightVelocityMotorConnected = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {
  }

  public default void stop() {
  }

  public default void setState(IntakeState state) {
  }

  public default void simulationPeriodic() {

  }

  public default void periodic() {

  }

  public default void runMotors(IntakeState currentState) {

  }

  public default void setVelocity(IntakeState currentState) {

  }

  public default void setPosition(IntakeState currentState) {

  }

  public default boolean isAtPosition(IntakeState state) {
    return false;
  }

  public default boolean isAtSpeed(IntakeState state) {
    return false;
  }
}
