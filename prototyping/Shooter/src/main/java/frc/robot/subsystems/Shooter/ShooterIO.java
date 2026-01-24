package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public double velocityLeftRPS = 0.0;
    public double velocityRightRPS = 0.0;

    
    public double velocityTopWheelRPS = 0.0;
    
    public double velocityBottomWheelRPS = 0.0;


    public double positionDeg = 0.0;
    public boolean velocityConnected = false;
    public boolean positionConnected = false;
  }

  default void updateInputs(ShooterIOInputs inputs) {
  }

   public default void setOutput(double shooterOutput, double backspinOutput, double angleOutput){}

  public default void setPosition(double position) {
  }

  public default void setVelocity(double velocity) {
  }
  public default void holdPosition(){
    
  }
  default void stop() {
  }
}