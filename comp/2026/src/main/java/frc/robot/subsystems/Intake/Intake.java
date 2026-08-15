package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private final IntakeIO io;
    private IntakeState intakeState = IntakeState.IDLE;

    public Intake(IntakeIO io){
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
    public void periodic() {
        switch(intakeState){
            case IDLE:
                io.stopRollers();
                io.stopPivot();
                break;
            default:
                io.setPivotAngle(intakeState.getPivotAngleDegrees());
                io.setRollerVelocity(intakeState.getRollerRPS());
                break;
        }
    }
}
