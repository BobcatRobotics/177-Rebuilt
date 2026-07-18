package frc.robot.subsystems.intake;

import org.bobcatrobotics.Framework.StateMachine.SubsystemState;
import org.littletonrobotics.junction.Logger;

public enum IntakeState implements SubsystemState {
    IDLE(0.0),
    STOW(0.0),
    DOWN(11.7),
    DOWN_AND_INTAKE(11.7, 400.0),
    DOWN_AND_OUTTAKE(1.0, -1.0),
    OUTTAKE(-400.0, true),
    INTAKE(400, true);

    private final double position;
    private final double rollerSpeed;

    IntakeState(double position) {
        this(position, 0.0);
    }

    IntakeState(double value, boolean rollerOnly) {
        this(0.0, value);
    }

    IntakeState(double position, double rollerSpeed) {
        this.position = position;
        this.rollerSpeed = rollerSpeed;
    }
    
    @Override
    public void onEnter() {
        Logger.recordOutput("IntakeState", "Entered " + getStateName());
    }
    
    @Override
    public void execute() {
        String name = getStateName();
        Logger.recordOutput("IntakeState", "In " + name);
    }

    @Override
    public void onExit() {
        Logger.recordOutput("IntakeState", "Exited " + getStateName());
    }

    public double getPosition() {
        return position;
    }

    public double getRollerSpeed() {
        return rollerSpeed;
    }

    public String getStateName() {
        return name();
    }
}