package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double velocityOfMainFlywheelLeftRPS = 0;
    public double velocityOfMainFlywheelRightRPS = 0;
    public double velocityOfbackspinWheelMotorLeftRPS = 0;
    public double velocityOfbackspinWheelMotorRightRPS = 0;
    public double velocityOfMainFlywheelOuterRightRPS = 0;
    public double velocityOfIntakeRPS = 0;
    public double accelerationOfMainFlywheelLeft = 0;
    public double accelerationOfMainFlywheelRight = 0;
    public double accelerationOfMainFlywheelOuterRight = 0;
    public double accelerationOfBackspinLeft = 0;
    public double accelerationOfIntake = 0;
    public double statorCurrentOfBackspinLeftAmps = 0;
    public double statorCurrentOfBackspinRightAmps = 0;
    public double statorCurrentOfMainFlywheelLeftAmps = 0;
    public double statorCurrentOfMainFlywheelRightAmps = 0;
    public double statorCurrentOfMainFlywheelOuterRightAmps = 0;
    public double statorCurrentOfIntakeAmps = 0;
    public boolean backspinWheelMotorRightConnected = false;
    public boolean backspinWheelMotorLeftConnected = false;
    public boolean shooterFlywheelInnerLeftConnected = false;
    public boolean shooterFlywheelInnerRightConnected = false;
    public boolean shooterFlywheelOuterRightConnected = false;
    public boolean shooterIntakeMotorConnected = false;
    public double outputOfBackspinLeftVolts = 0;
    public double outputOfBackspinRightVolts = 0;
    public double outputOfMainFlywheelLeftVolts = 0;
    public double outputOfMainFlywheelRightVolts = 0;
    public double outputOfMainFlywheelOuterRightVolts = 0;
    public double outputOfIntakeVolts = 0;
  }

  default void updateInputs(ShooterIOInputs inputs) {
  }

  public default void setVelocity(ShooterState desiredState) {
  }

  public default void setVelocity(double ShooterSpeed, double ShooterBackspinSpeedLeft,
      double ShooterBackspinSpeedRight, double shooterIntakeSpeed) {
  }

  public default void setMainWheelSpeed(double shooterFlywheelSpeedInRPS) {
  }

  public default void setBackspinSpeedRight(double shooterBackspinSpeedInRPS) {
  }

  public default void setBackspinSpeedLeft(double shooterBackspinSpeedInRPS) {
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

  public default void stopBackspinWheel() {

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


  public default void runCharacterization_Backspin(double output) {
  }

  public default double getFFCharacterizationVelocity_Backspin() {
    return 0.0;
  }

    public default void runCharacterization_Intake(double output) {
  }

  public default double getFFCharacterizationVelocity_Intake() {
    return 0.0;
  }
}