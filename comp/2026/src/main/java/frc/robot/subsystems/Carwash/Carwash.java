package frc.robot.subsystems.Carwash;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Carwash extends SubsystemBase{
    CarwashState currentState = CarwashState.IDLE;
    private final CarwashIO io;

    public Carwash(CarwashIO io){
        this.io = io;
    }
    public void periodic(){
        switch(currentState){
            case IDLE -> {
                io.stop();
                break;
            }
            case INTAKE -> {
                io.setRPS(currentState.getRPS());
                break;
            }
            case OUTTAKE ->{
                io.setRPS(currentState.getRPS());
                break;
            }
        }
    }
    public void stop(){
    io.stop();
  }

  public void setState(CarwashState state){
    this.currentState = state;
  }

  public CarwashState getState(){
    return currentState;
  }
}
