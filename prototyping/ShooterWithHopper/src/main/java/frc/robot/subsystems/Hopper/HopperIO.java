package frc.robot.subsystems.Hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

  @AutoLog
  class HopperIOInputs {
    public double velocityOfHopperTopRPS = 0.0;
    public double statorCurrentOfHopperTopAmps = 0.0;
    public double outputOfHopperTopVolts = 0.0;
    public double accelerationOfHopperTop = 0.0;
    public boolean hopperTopConnected = false;

    public double velocityOfHopperBottomRPS = 0.0;
    public double statorCurrentOfHopperBottomAmps = 0.0;
    public double outputOfHopperBottomVolts = 0.0;
    public double accelerationOfHopperBottom = 0.0;
    public boolean hopperBottomConnected = false;
  }

  default void updateInputs(HopperIOInputs inputs) {
  }


  public default void setVelocity(HopperState desiredState) {
  }

  public default void setVelocity(double topVelocity, double bottomVelocity){
  }

  public default void setTopSpeed(double speed){

  }

  public default void setBottomSpeed(double speed){
    
  }

  public default void holdPosition() {

  }

  public default void periodic() {

  }

  default void stop() {
  }

  public default void stopTop() {
  }
 public default void stopBottom() {
  }

  public default void simulationPeriodic() {

  }

  public default void runCharacterization_Hopper(double output) {
  }

  public default double getFFCharacterizationVelocity_Hopper() {
    return 0.0;
  }


}