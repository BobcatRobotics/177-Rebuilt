package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double velocityOfDumperLeftUpRPS = 0;
    public double velocityOfDumperLeftDownRPS = 0;
    // public double velocityOfHoodWheelMotorLeftRPS = 0;
    // public double velocityOfHoodWheelMotorRightRPS = 0;
    public double velocityOfDumperRightUpRPS = 0;
    public double velocityOfDumperRightDownRPS = 0;

    public double accelerationOfDumperLeftUp = 0;
    public double accelerationOfDumperLeftDown = 0;
    public double accelerationOfDumperRightUp = 0;
    public double accelerationOfDumperRightDown = 0;
    public double accelerationOfAdjustableHood = 0;
    // public double accelerationOfHoodLeft = 0;
    public double statorCurrentOfDumperLeftUp = 0;
    // public double statorCurrentOfHoodRightAmps = 0;
    public double statorCurrentOfDumperLeftDown = 0;
    public double statorCurrentOfDumperRightUp = 0;
    public double statorCurrentOfDumperRightDown = 0;
    // public double statorCurrentOfMainFlywheelOuterLeftAmps = 0;
    // public boolean HoodWheelMotorRightConnected = false;
    // public boolean HoodWheelMotorLeftConnected = false;
    public boolean DumperLeftUpConnected = false;
    public boolean DumperLeftDownConnected = false;
    public boolean DumperRightUpConnected = false;
    public boolean DumperRightDownConnected = false;
    // public double outputOfHoodLeftVolts = 0;
    // public double outputOfHoodRightVolts = 0;
    public double outputOfDumperLeftUpVolts = 0;
    public double outputOfDumperLeftDownVolts = 0;
    public double outputOfDumperRightUpVolts = 0;
    public double outputOfDumperRightDownVolts = 0;
    public double outputOfAdjustableHoodVolts = 0;

    public double statorCurrentOfAdjustableHoodPositionAmps = 0; 
    public boolean adjustableHoodConnected = false; 
    public double velocityOfAdjustableHoodPositionRPS = 0;
    public double positionOfAdjustableHood = -1; 

  }

  default void updateInputs(ShooterIOInputs inputs) {
  }

  public default void setShot(ShooterState desiredState) {
  }

  public default void setVelocity(double dumperLeftSpeed, double dumperRightSpeed,
      double adjustableHoodPositon) {
  }

  public default void setDumperLeftSpeed(double speed) {
  }

  public default void setDumperRightSpeed(double speed) {
  }

  public default void setAdjustableHoodPosition(double position){

  }

  // public default void setHoodSpeedLeft(double shooterHoodSpeedInRPS) {
  // }

  public default void holdPosition() {

  }

  public default void periodic() {

  }

  default void stop() {
  }

  public default void stopDumperLeft() {
  }

  public default void stopDumperRight() {

  }

  public default void stopAdjustableHood(){

  }

  public default void simulationPeriodic() {

  }

  public default void runCharacterization_Flywheel(double output) {
  }

  public default double getFFCharacterizationVelocity_Flywheel() {
    return 0.0;
  }

  // public default void runCharacterization_Hood(double output) {
  // }

  // public default double getFFCharacterizationVelocity_Hood() {
  //   return 0.0;
  // }

  // public default double getVelocityHood() {
  //   return 0.0;
  // }

  // public default double getVelocityMainFlywheel() {
  //   return 0.0;
  // }
}