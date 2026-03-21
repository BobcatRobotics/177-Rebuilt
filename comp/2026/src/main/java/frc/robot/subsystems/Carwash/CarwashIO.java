package frc.robot.subsystems.Carwash;

import org.littletonrobotics.junction.AutoLog;

public interface CarwashIO {

  @AutoLog
  class CarwashIOInputs {
    public double velocityOfIntakeRPS = 0;
    public double accelerationOfIntake = 0;
    public double statorCurrentOfIntakeAmps = 0;
    public boolean shooterIntakeMotorConnected = false;
    public double outputOfIntakeVolts = 0;
  }

  default void updateInputs(CarwashIOInputs inputs) {
  }

  public default void setVelocity(CarwashState desiredState) {
  }

  public default void setVelocity(double shooterIntakeSpeed) {
  }

  public default void setIntakeSpeed(double shooterIntakeSpeedInRPS) {
  }

  public default void holdPosition() {

  }

  public default void periodic() {

  }

  default void stop() {
  }

  public default void stopIntakeWheel() {
  }

  public default void simulationPeriodic() {

  }

  public default void runCharacterization_Intake(double output) {
  }

  public default double getFFCharacterizationVelocity_Intake() {
    return 0.0;
  }

}