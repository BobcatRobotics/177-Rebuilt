package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterState.State;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private ShooterState desiredState;

  public Shooter(ShooterIO io) {
    this.io = io;
    desiredState = new ShooterState();
    desiredState.setState(State.IDLE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/State", desiredState.currentState);
  }

  public void setState(ShooterState.State state){
    setPosition(state);
    setVelocity(state);
  }

  public void setManualOutput(double shooterOutput, double backspinOutput, double angleOutput){
    io.setOutput( shooterOutput, backspinOutput, angleOutput);
  }  
  public void setPosition(ShooterState.State state) {
    desiredState.setState(state);
    io.setPosition(desiredState.getOutput().position);
  }

  public void setVelocity(ShooterState.State state) {
    desiredState.setState(state);
    io.setVelocity(desiredState.getOutput().speed);
  }
    public void setVelocity(double velocity) {
    io.setVelocity(velocity);
  }

  public void holdPosition(){
    io.holdPosition();
  }

  public void stop() {
    io.stop();
  }
}