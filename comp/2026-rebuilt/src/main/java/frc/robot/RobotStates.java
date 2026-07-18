package frc.robot;

import org.bobcatrobotics.Framework.StateMachine.RobotState;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.carwash.CarwashState;
import frc.robot.subsystems.hopper.HopperState;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.shooter.ShooterState;

public enum RobotStates implements RobotState {
    IDLE(ShooterState.IDLE, CarwashState.IDLE, HopperState.IDLE, IntakeState.IDLE),
    INTAKEDOWN(IntakeState.DOWN),
    INTAKESTOW(IntakeState.STOW),
    INTAKEDOWNANDINTAKE(IntakeState.DOWN_AND_INTAKE),
    INTAKEDOWNANDOUTTAKE(IntakeState.DOWN_AND_OUTTAKE),
    SPINUPSHOOTSEQ(ShooterState.INTERPOLATED, CarwashState.OUTTAKE, HopperState.SPINUP, IntakeState.INTAKE),
    FEEDANDSHOOTSEQ(ShooterState.INTERPOLATED, CarwashState.FEED, HopperState.INTAKE, IntakeState.INTAKE);

    private CarwashState currentCarwashState;
    private ShooterState currentShooterState;
    private IntakeState currentIntakeState;
    private HopperState currentHopperState;

    RobotStates(IntakeState intakeState){

    }

    RobotStates(ShooterState shooterState, CarwashState carwashState, HopperState hopperState,
            IntakeState intakeState) {
        this.currentShooterState = shooterState;
        this.currentCarwashState = carwashState;
        this.currentHopperState = hopperState;
        this.currentIntakeState = intakeState;
    }

    @Override
    public void onEnter() {
        Logger.recordOutput("CurrentState" , "Entered "+ name());
    }

    @Override
    public void onExit() {
        Logger.recordOutput("PreviousState" , "Exited " + name());
    }

    public ShooterState getShooterState() {
        return currentShooterState;
    }

    public CarwashState getCarwashState() {
        return currentCarwashState;
    }

    public IntakeState getIntakeState() {
        return currentIntakeState;
    }

    public HopperState getHopperState() {
        return currentHopperState;
    }
}
