package frc.robot.subystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  class ClimberIOInputs {
    public double velocityOfClimberRPS = 0.0;
    public double statorCurrentOfClimberAmps = 0.0;
    public double outputOfClimberVolts = 0.0;
    public double accelerationOfClimber = 0.0;
    public boolean climberConnected = false;
    public double climberPosition = -1;
  }

  default void updateInputs(ClimberIOInputs inputs) {
  }


  public default void setPosition(ClimberState desiredState) {
  }

  public default void holdPosition() {

  }

  public default void periodic() {

  }

  default void stop() {
  }

  public default void simulationPeriodic() {

  }

  public default void runCharacterization_Climber(double output) {
  }

  public default double getFFCharacterizationVelocity_Climber() {
    return 0.0;
  }


}