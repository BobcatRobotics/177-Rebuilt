package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Turret.TurretState.State;

public class Turret extends SubsystemBase {

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private TurretState desiredState;

  public Turret(TurretIO io) {
    this.io = io;
    desiredState = new TurretState();
    desiredState.setState(State.IDLE);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/State", desiredState.currentState);
  }

  public void setState(TurretState.State state){
    setPosition(state);
  }

  public void setPosition(TurretState.State state) {
    desiredState.setState(state);
    io.setPosition(desiredState.getOutput().position);
  }


  public void holdPosition(){
    io.holdPosition();
  }

  public void stop() {
    io.stop();
  }
}