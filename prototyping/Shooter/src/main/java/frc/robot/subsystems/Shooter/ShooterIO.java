package frc.robot.subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.Shooter.Modules.ModuleType;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double velocityOfMainFlywheelInnerRPMLeft = 0.0;
     public double velocityOfMainFlywheelInnerRPMRight = 0.0;
    public double velocityOfbackspinWheelMotorRPM = 0.0;
    public double velocityOfIntakeWheelMotorRPM = 0.0;


    public double velocityOfMainFlywhelInnerRPMErrorRight = 0.0;
    public double velocityOfMainFlywhelInnerRPMErrorLeft = 0.0;
    public double velocityOfbackspinWheelMotorRPMError = 0.0;
    public double velocityOfIntakeWheelMotorRPMError = 0.0;

    public double outputOfMainFlywhelInnerLeft = 0.0;
    public double outputOfMainFlywhelInnerRight = 0.0;
    public double outputOfbackspinWheelMotor = 0.0;
     public double outputOfIntakeWheelInner = 0.0;

    public double accelerationOfMainFlywhelInnerLeft = 0.0;
    public double accelerationOfMainFlywhelInnerRight = 0.0;
    public double accelerationOfbackspinWheelMotor = 0.0;
    public double accelerationOfIntakeWheelMotor = 0.0;

    public double mainFlywheelInnerStatorCurrentLeft = 0.0;
    public double mainFlywheelInnerStatorCurrentRight = 0.0;
    public double mainBackspinStatorCurrent = 0.0;
    public double mainIntakeStatorCurrent = 0.0;

    public boolean mainFlywhelInnerConectedLeft = false;
    public boolean mainFlywhelInnerConectedRight = false;
    public boolean backspinConnected = false;
    public boolean intakeConnected = false;
  }

  default void updateInputs(ShooterIOInputs inputs) {
  }

  public default void setVelocity(ShooterState desiredState) {
  }

  public default void setVelocity(double ShooterSpeed, double ShooterBackspinSpeed,double shooterIntakeSpeed) {
  }

  public default void setMainWheelSpeed(double shooterFlywheelSpeedInRPM) {
  }

  public default void setBackspinSpeed(double shooterBackspinSpeedInRPM) {
  }

  public default void setIntakeSpeed(double shooterIntakeSpeedInRPM) {
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

  public default List<ModuleType> getModuleTypes() {
    return new ArrayList<ModuleType>();
  }
}