package frc.robot;

import frc.robot.subsystems.Carwash.Carwash;
import frc.robot.subsystems.Carwash.CarwashState;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodState;
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
    private final Hood hood;
    private final Carwash carwash;
    
    private RobotStateType state = RobotStateType.IDLE;


    public RobotState(
        Hopper hopper, 
        Intake intake, 
        Shooter shooter,
        Hood hood,
        Carwash carwash){
            this.hopper = hopper;
            this.intake = intake;
            this.shooter = shooter;
            this.hood = hood;
            this.carwash = carwash;
    }

    public void setState(RobotStateType newState) {
        state = newState;

        switch (state) {
            case IDLE:
                hopper.setState(HopperState.IDLE);
                intake.setState(IntakeState.IDLE);
                shooter.setState(ShooterState.IDLE);
                hood.setState(HoodState.IDLE);
                carwash.setState(CarwashState.IDLE);
                break;

            case INTAKING:
                hopper.setState(HopperState.FORWARD);
                intake.setState(IntakeState.INTAKE);
                shooter.setState(ShooterState.IDLE);
                break;

            case SHOOTING:
                hood.setState(HoodState.DEPLOYED);
                carwash.setState(CarwashState.FEED);
                shooter.setState(ShooterState.SHOOT);
                hopper.setState(HopperState.FORWARD);
                intake.setState(IntakeState.INTAKE);
                break;

            case EJECTING:
                hood.setState(HoodState.IDLE);
                carwash.setState(CarwashState.REVERSE);
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
