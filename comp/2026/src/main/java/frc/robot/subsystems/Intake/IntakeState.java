package frc.robot.subsystems.Intake;

public class IntakeState {
    public enum State{
        IDLE(0),
        ROLLING_IN(0),
        ROLLING_OUT(0),
        INTAKE_IN(0),
        INTAKE_OUT(0);
        
        public double rps;

        State(double rps){
            this.rps = rps;
        }
    }
    public double rps;

    IntakeState(){
        
    }

    public void intake_in(){

    }

    public void intake_out(){

    }
    public void run_roller(){

    }
}
