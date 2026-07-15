package frc.robot.subsystems.hopper;

import org.bobcatrobotics.Framework.StateMachine.SubsystemState;
import org.littletonrobotics.junction.Logger;

public enum HopperState implements SubsystemState{
    IDLE(0.0),
    SPINUP(-30),
    INTAKE(60),
    OUTTAKE(-50);
    
    private final double rollerSpeed;

    HopperState(double velocity){
        this.rollerSpeed = velocity;
    }

    @Override
    public void onEnter() {
        Logger.recordOutput("Hopper/"+name(),"Entered");
    }

    @Override
    public void onExit() {
        Logger.recordOutput("Hopper/"+name(),"Exited");
    }

    public double getRollerSpeed() {
        return rollerSpeed;
    }
}