package frc.robot.subsystems.shooter;

import org.bobcatrobotics.Framework.StateMachine.SubsystemState;
import org.littletonrobotics.junction.Logger;

public enum ShooterState  implements SubsystemState{
    IDLE(0.0, 0.0),
    MANUAL_SPINUP(1.0, 0.0),
    MANUL_SHOOT(1.0, 0.5),

    // Uses interpolation instead of fixed values
    INTERPOLATED();

    private final double rollerSpeed;
    private final double position;
    private final boolean interpolated;

    ShooterState(double rollerSpeed, double position) {
        this.rollerSpeed = rollerSpeed;
        this.position = position;
        this.interpolated = false;
    }

    ShooterState() {
        this.rollerSpeed = 0.0;
        this.position = 0.0;
        this.interpolated = true;
    }

    @Override
    public void onEnter() {
        Logger.recordOutput("Shooter/"+name(),"Entered");
    }

    @Override
    public void onExit() {
        Logger.recordOutput("Shooter/"+name(),"Exited");
    }


    public boolean isInterpolated() {
        return interpolated;
    }

    public double getPosition() {
        return position;
    }

    public double getRollerSpeed() {
        return rollerSpeed;
    }
}