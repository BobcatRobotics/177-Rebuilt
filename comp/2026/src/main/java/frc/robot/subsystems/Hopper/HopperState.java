package frc.robot.subsystems.Hopper;


public enum HopperState {
  IDLE(0),
  FORWARD(60),
  REVERSE(60);

  private final double rps;
    
  HopperState(double rps){
    this.rps = rps;
  }
    
  public double getRPS() {
    return rps;
  }
  }
/*  State currentState = State.IDLE;
  
  public HopperState(){}

  public void setState(State state){
    this.currentState = state;
  } */



