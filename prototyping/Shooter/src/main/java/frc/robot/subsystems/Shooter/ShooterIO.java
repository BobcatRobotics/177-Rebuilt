package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double velocityOfMainFlywhelLeftRPS = 0.0;
    public double velocityOfMainFlywhelRightRPS = 0.0;
    public double velocityOfbackspinWheelMotorRPS = 0.0;
    public double velocityOfIntakeWheelRPS = 0.0;
    public boolean mainFlywhelLeftConnected = false;
    public boolean mainFlywhelRightConnected = false;
    public boolean backspinConnected = false;
    public boolean intakeConnected = false;
  }

  default void updateInputs(ShooterIOInputs inputs) {
  }

  public default void setVelocity(ShooterState desiredState) {
  }

  public default void holdPosition() {

  }
  public default void periodic(){
    
  }

  default void stop() {
  }
}