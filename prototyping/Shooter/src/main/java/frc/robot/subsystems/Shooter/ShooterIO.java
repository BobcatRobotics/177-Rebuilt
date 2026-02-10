package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double velocityOfMainFlywheelInnerRPS = 0.0;
    public double velocityOfbackspinWheelMotorRPS = 0.0;
    public double velocityOfIntakeWheelMotorRPS = 0.0;


    public double velocityOfMainFlywhelInnerRPSError = 0.0;
    public double velocityOfbackspinWheelMotorRPSError = 0.0;
    public double velocityOfIntakeWheelMotorRPSError = 0.0;

    public double outputOfMainFlywhelInner = 0.0;
    public double outputOfbackspinWheelMotor = 0.0;
     public double outputOfIntakeWheelInner = 0.0;

    public double accelerationOfMainFlywhelInner = 0.0;
    public double accelerationOfbackspinWheelMotor = 0.0;
    public double accelerationOfIntakeWheelMotor = 0.0;

    public double mainFlywheelInnerStatorCurrent = 0.0;
    public double mainBackspinStatorCurrent = 0.0;
    public double mainIntakeStatorCurrent = 0.0;

    public boolean mainFlywhelInnerConected = false;
    public boolean backspinConnected = false;
    public boolean intakeConnected = false;
  }

  default void updateInputs(ShooterIOInputs inputs) {
  }

  public default void setVelocity(ShooterState desiredState) {
  }

  public default void setVelocity(double ShooterSpeed, double ShooterBackspinSpeed,double shooterIntakeSpeed) {
  }

  public default void setMainWheelSpeed(double shooterFlywheelSpeed) {
  }

  public default void setBackspinSpeed(double shooterBackspinSpeed) {
  }

  public default void setIntakeSpeed(double shooterIntakeSpeed) {
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

  public default void simulationPeriodic(){
    
  }

  public default String getName(){
    return "";
  }
}