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
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/State", desiredState.getCurrentState());

  }

  public void setState(ShooterState state) {
    desiredState = state;
    setVelocity(desiredState.getCurrentState());
  }

  private void setVelocity(ShooterState.State state) {
    desiredState.setState(state);
    io.setVelocity(desiredState);
  }

  private void setVelocity(double ShooterSpeed, double ShooterIntakeSpeed, double ShooterBackspinSpeed) {
    io.setVelocity(ShooterSpeed, ShooterIntakeSpeed, ShooterBackspinSpeed);
  }

  public void setMainWheelSpeed(double shooterFlywheelSpeed) {
    io.setMainWheelSpeed(shooterFlywheelSpeed);
  }

  public void setBackspinSpeed(double shooterBackspinSpeed) {
    io.setBackspinSpeed(shooterBackspinSpeed);
  }

  public void setIntakeSpeed(double shooterIntakeSpeed) {
    io.setIntakeSpeed(shooterIntakeSpeed);
  }

  public void holdPosition() {
    io.holdPosition();
  }

  public void stop() {
    io.stop();
  }

  public void stopMainWheel(){
    io.stopMainWheel();
  }
  public void stopBackspinWheel(){
    io.stopBackspinWheel();

  }
  public void stopIntakeWheel(){
    io.stopBackspinWheel();
  }
}