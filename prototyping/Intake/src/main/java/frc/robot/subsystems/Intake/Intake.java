package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeState.State;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private IntakeState desiredState;

  public Intake(IntakeIO io) {
    this.io = io;
    desiredState = new IntakeState();
    desiredState.setState(State.IDLE);
  }

  public void applyState(){
    desiredState = new IntakeState();
    desiredState.setState(State.IDLE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/State", desiredState.currentState);
  }

  public void setState(IntakeState.State state){
    setVelocity(state);
  }

  public void setVelocity(IntakeState.State state) {
    desiredState.setState(state);
    io.setVelocity(desiredState.getOutput().speed);
  }

  public void stop() {
    io.stop();
  }
}