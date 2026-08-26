package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    IntakeState currentState = IntakeState.IDLE;
    private final IntakeIO io;

    public Intake(IntakeIO io){
      this.io = io;
    }

    public void periodic(){
        switch (currentState) {
      case IDLE -> {
        io.stop();
      }
      case ROLLING_IN -> {
        io.setRPS(currentState.getRPS()); //set arguments for rps
      }
      case ROLLING_OUT -> {
        io.setRPS(currentState.getRPS()); //set arguments for rps
      }
      case INTAKE_IN -> {
        io.setRPS(currentState.getRPS());  //set arguments for rps
      }
      case INTAKE_OUT -> {
        io.setRPS(currentState.getRPS());  //set arguments for rps
      }

    }
}
  public void stop(){
    io.stop();
  }

  public void setState(IntakeState state){
    this.currentState = state;
  }

  public IntakeState getState(){
    return currentState;
  }

public void setNeturalCoast() {
    //requires implementation to change motor contorl config to neutral coast mode
}

public void setNeturalBrake() {
    //requires implementation to change motor contorl config to neutral break mode
}
}
