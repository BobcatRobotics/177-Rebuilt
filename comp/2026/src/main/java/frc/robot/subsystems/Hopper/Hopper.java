package frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase{

    HopperState currentState = HopperState.IDLE;
    private final HopperIO io;

    public Hopper(HopperIO io){
      this.io = io;
    }

    public void periodic(){
      io.periodic();

        switch(currentState){
      case IDLE -> {
        io.stop();
        break;
      }
      case FORWARD -> {
        io.setRPS(currentState.getRPS());
        break;
      }
      case REVERSE -> {
        io.setRPS(currentState.getRPS());
        break;
      }

    }
  }
  public void stop(){
    io.stop();
  }

  public void setState(HopperState state){
    this.currentState = state;
  }

  public HopperState getState(){
    return currentState;
  }
  @Override
  public void simulationPeriodic(){
    io.simulationPeriodic();
  }

}
