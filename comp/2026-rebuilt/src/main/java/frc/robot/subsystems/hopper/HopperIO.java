package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.carwash.CarwashState;

public interface HopperIO {

  @AutoLog
  class HopperIOInputs {
    public double velocityOfHopperTopRPS = 0.0;
    public double statorCurrentOfHopperTopAmps = 0.0;
    public double torqueCurrentHopperTopAmps = 0.0;
    public double outputOfHopperTopVolts = 0.0;
    public double accelerationOfHopperTop = 0.0;
    public boolean hopperTopConnected = false;
  }
  public default void updateInputs(HopperIOInputs inputs) {
  }

  public default void stop() {
  }

  public default void setState(HopperState state) {
  }

  public default void simulationPeriodic() {

  }

  public default void periodic() {

  }
    public default void runMotors(HopperState currentState) {

    }
}