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
  }

  public void applyState(){

    desiredState = new ShooterState();
    desiredState.setState(State.IDLE);
  }



  @Override
  public void periodic() {
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/inputs", inputs);
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

  private void setVelocity(double shooterSpeed, double shooterBackspinSpeedLeft,double shooterBackspinSpeedRight, double shooterIntakeSpeed) {
    io.setVelocity(shooterSpeed, shooterBackspinSpeedLeft,shooterBackspinSpeedRight, shooterIntakeSpeed);
  }

  public void setMainWheelSpeed(double shooterFlywheelSpeed) {
    io.setMainWheelSpeed(shooterFlywheelSpeed);
  }

  public void setBackspinSpeedLeft(double shooterBackspinSpeed) {
    io.setBackspinSpeedLeft(shooterBackspinSpeed);
  }
    public void setBackspinSpeedRight(double shooterBackspinSpeed) {
    io.setBackspinSpeedRight(shooterBackspinSpeed);
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

  public void stopMainWheel() {
    io.stopMainWheel();
  }

  public void stopBackspinWheel() {
    io.stopBackspinWheel();

  }

  public void stopIntakeWheel() {
    io.stopBackspinWheel();
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }
}