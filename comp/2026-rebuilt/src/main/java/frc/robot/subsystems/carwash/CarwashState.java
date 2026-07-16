package frc.robot.subsystems.carwash;

import org.bobcatrobotics.Framework.StateMachine.SubsystemState;
import org.littletonrobotics.junction.Logger;

public enum CarwashState implements SubsystemState{
    IDLE(0.0), 
    FEED(80.0),
    OUTTAKE(-20.0);

    private final double carwashSpeed;

    CarwashState(double velocity) {
        this.carwashSpeed = velocity;
    }

    @Override
    public void onEnter() {
        Logger.recordOutput("Carwash/"+name(),"Entered");
    }

    @Override
    public void onExit() {
        Logger.recordOutput("Carwash/"+name(),"Exited");
    }


    public double getCarwashSpeed() {
        return carwashSpeed;
    }
}
