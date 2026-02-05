package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double velocityOfMainFlywhelLeftRPS = 0.0;
    public double velocityOfMainFlywhelRightRPS = 0.0;
    public double velocityOfbackspinWheelMotorRPS = 0.0;
    public double velocityOfIntakeWheelRPS = 0.0;

    public double velocityOfMainFlywhelLeftRPSError = 0.0;
    public double velocityOfMainFlywhelRightRPSError = 0.0;
    public double velocityOfbackspinWheelMotorRPSError = 0.0;
    public double velocityOfIntakeWheelRPSError = 0.0;

    public double outputOfMainFlywhelLeft = 0.0;
    public double outputOfMainFlywhelRight = 0.0;
    public double outputOfbackspinWheelMotor = 0.0;
    public double outputOfIntakeWheel = 0.0;

    public double accelerationOfMainFlywhelLeft = 0.0;
    public double accelerationOfMainFlywhelRight = 0.0;
    public double accelerationOfbackspinWheelMotor = 0.0;
    public double accelerationOfIntakeWheel = 0.0;

    public double mainFlywheelLeftStatorCurrent = 0.0;
    public double mainFlywheelRightStatorCurrent = 0.0;
    public double mainBackspinStatorCurrent = 0.0;
    public double mainIntakeStatorCurrent = 0.0;

    public boolean mainFlywhelLeftConnected = false;
    public boolean mainFlywhelRightConnected = false;
    public boolean backspinConnected = false;
    public boolean intakeConnected = false;
  }

  default void updateInputs(ShooterIOInputs LeftInputs, ShooterIOInputs RightInputs) {
  }

  public default void setVelocity(ShooterState desiredState) {
  }

  public default void setVelocity(double ShooterSpeed, double ShooterIntakeSpeed, double ShooterBackspinSpeed) {
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
}