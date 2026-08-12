package frc.robot;

import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class RobotState  {
    private final Hopper hopper;
    private final Intake intake;
    private final Shooter shooter;
    
    private RobotStateType state = RobotStateType.IDLE;


    public RobotState(
        Hopper hopper, 
        Intake intake, 
        Shooter shooter){
            this.hopper = hopper;
            this.intake = intake;
            this.shooter = shooter;
    }

    public void setState(RobotStateType newState) {
        state = newState;

        switch (state) {
            case IDLE:
                hopper.setState(HopperState.IDLE);
                intake.setState(IntakeState.IDLE);
                shooter.setState(ShooterState.IDLE);
                break;

            case INTAKING:
                hopper.setState(HopperState.FORWARD);
                intake.setState(IntakeState.INTAKE);
                shooter.setState(ShooterState.IDLE);
                break;

            case SHOOTING:
                shooter.setState(ShooterState.SHOOT);
                hopper.setState(HopperState.FORWARD);
                intake.setState(IntakeState.INTAKE);
                break;

            case EJECTING:
                shooter.setState(ShooterState.IDLE);
                hopper.setState(HopperState.REVERSE);
                intake.setState(IntakeState.EJECT);
                break;
        }
    }

    public RobotStateType getState() {
        return state;
    }
}
