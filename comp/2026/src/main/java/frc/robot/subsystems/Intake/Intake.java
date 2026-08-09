package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private final IntakeIO io;
    private IntakeState intakeState = IntakeState.IDLE;

    Intake(IntakeIO io){
        this.io = io;
    }

    public void setState(IntakeState state)
    {
        this.intakeState = state;
    }

    public IntakeState getState()
    {
        return this.intakeState;
    }

    @Override
    public void periodic(){
        io.setVelocity(intakeState.getRPS());
    }
}
