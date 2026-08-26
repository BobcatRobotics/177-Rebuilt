package frc.robot.subsystems.Shooter;

public class ShooterIOReal implements ShooterIO{
    ShooterState currentState = ShooterState.IDLE;
    private final ShooterIO io;

    public ShooterIOReal(ShooterIO io){
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
