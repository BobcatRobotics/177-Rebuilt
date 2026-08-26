package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    ShooterState currentState = ShooterState.IDLE;
    private final ShooterIO io;

    public Shooter(ShooterIO io){
      this.io = io;
    }

    public void autoPeriodic(){
      
    }

    public void periodic(){
      io.periodic();

        switch(currentState){
      case IDLE -> {
        io.stop();
        break;
      }
      case SHOOT -> {
        io.setRPS(currentState.getRPS());
        break;
      }
      case REVERSE_SHOOT -> {
        io.setRPS(currentState.getRPS());
        break;
      }
      case SPIN_UP -> {
        io.setRPS(currentState.getRPS());
        break;
      }

    }
  }
  public void stop(){
    io.stop();
  }

  public void setState(ShooterState state){
    this.currentState = state;
  }

  public ShooterState getState(){
    return currentState;
  }

}