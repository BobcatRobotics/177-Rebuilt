package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public class IntakeIOInputs {
    public double velocityOfIntakePositionRPS = 0.0;
    public double statorCurrentOfIntakePositionAmps = 0.0;
    public double outputOfIntakePositionVolts = 0.0;
    public double accelerationOfIntakePosition = 0.0;
    public boolean velocityConnected = false;

    public double velocityOfIntakeSpeedRPS = 0.0;
    public double statorCurrentOfIntakeSpeedAmps = 0.0;
    public double outputOfIntakeSpeedVolts = 0.0;
    public double accelerationOfIntakeSpeed = 0.0;
    public boolean positionConnected = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {
  }

  public default void setVelocity(double velocity) {
  }

  public default void setVelocity(IntakeState desiredState) {
  }

  public default void setPosition(IntakeState desiredState) {
  }

  public default void setPosition(double pos) {
  }

  public default double getVelocity() {
    return 0.0;
  }

  public default void holdPosition(){

  }
  public default void stop() {
  }

  public default void stopRollerWheel() {
  }

  public default void stopPivotMotor() {
  }

  public default void stopBottom() {
  }

  public default void periodic() {
  }
    public default void simulationPeriodic() {
  }

  public default void runCharacterization_Intake(double output) {
  }

  public default double getFFCharacterizationVelocity_Intake() {
    return 0.0;
  }

}