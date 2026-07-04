package frc.robot.subsystems.intake;

import org.bobcatrobotics.Framework.StateMachine.StateMachine;
import org.bobcatrobotics.Framework.StateMachine.Transition;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.Idle;

public class IntakeStateMachine {
    public enum states {
        IDLE,DOWN,UP,INTAKEBALLS,OUTAKEBALLS,HOLD
    }
    public states currentState = states.IDLE;
    public StateMachine internalStateMachine;
    public IntakeStateMachine(){
        internalStateMachine = new StateMachine<states>(currentState, "Intake/StateMachine");
    }
}
