package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double velocityOfMainFlywheelLeftRPS = 0;
    public double velocityOfMainFlywheelRightRPS = 0;
    public double velocityOfHoodWheelMotorLeftRPS = 0;
    public double velocityOfHoodWheelMotorRightRPS = 0;
    public double velocityOfMainFlywheelOuterRightRPS = 0;
    public double velocityOfMainFlywheelOuterLeftRPS = 0;
    public double velocityOfIntakeRPS = 0;
    public double accelerationOfMainFlywheelLeft = 0;
    public double accelerationOfMainFlywheelRight = 0;
    public double accelerationOfMainFlywheelOuterRight = 0;
    public double accelerationOfMainFlywheelOuterLeft = 0;
    public double accelerationOfHoodLeft = 0;
    public double accelerationOfIntake = 0;
    public double statorCurrentOfHoodLeftAmps = 0;
    public double statorCurrentOfHoodRightAmps = 0;
    public double statorCurrentOfMainFlywheelLeftAmps = 0;
    public double statorCurrentOfMainFlywheelRightAmps = 0;
    public double statorCurrentOfMainFlywheelOuterRightAmps = 0;
    public double statorCurrentOfMainFlywheelOuterLeftAmps = 0;
    public double statorCurrentOfIntakeAmps = 0;
    public boolean HoodWheelMotorRightConnected = false;
    public boolean HoodWheelMotorLeftConnected = false;
    public boolean shooterFlywheelInnerLeftConnected = false;
    public boolean shooterFlywheelInnerRightConnected = false;
    public boolean shooterFlywheelOuterRightConnected = false;
    public boolean shooterFlywheelOuterLeftConnected = false;
    public boolean shooterIntakeMotorConnected = false;
    public double outputOfHoodLeftVolts = 0;
    public double outputOfHoodRightVolts = 0;
    public double outputOfMainFlywheelLeftVolts = 0;
    public double outputOfMainFlywheelRightVolts = 0;
    public double outputOfMainFlywheelOuterRightVolts = 0;
    public double outputOfMainFlywheelOuterLeftVolts = 0;
    public double outputOfIntakeVolts = 0;
  }

  default void updateInputs(ShooterIOInputs inputs) {
  }

  public default void setVelocity(ShooterState desiredState) {
  }

  public default void setVelocity(double ShooterSpeed, double ShooterHoodSpeedLeft,
      double ShooterHoodSpeedRight, double shooterIntakeSpeed) {
  }

  public default void setMainWheelSpeed(double shooterFlywheelSpeedInRPS) {
  }

  public default void setHoodSpeedRight(double shooterHoodSpeedInRPS) {
  }

  public default void setHoodSpeedLeft(double shooterHoodSpeedInRPS) {
  }

  public default void setIntakeSpeed(double shooterIntakeSpeedInRPS) {
  }

  public default void holdPosition() {

  }

  public default void periodic() {

  }

  default void stop() {
  }

  public default void stopMainWheel() {
  }

  public default void stopHoodWheel() {

  }

  public default void stopIntakeWheel() {
  }

  public default void simulationPeriodic() {

  }

  public default void runCharacterization_Flywheel(double output) {
  }

  public default double getFFCharacterizationVelocity_Flywheel() {
    return 0.0;
  }


  public default void runCharacterization_Hood(double output) {
  }

  public default double getFFCharacterizationVelocity_Hood() {
    return 0.0;
  }

    public default void runCharacterization_Intake(double output) {
  }

  public default double getFFCharacterizationVelocity_Intake() {
    return 0.0;
  }
}