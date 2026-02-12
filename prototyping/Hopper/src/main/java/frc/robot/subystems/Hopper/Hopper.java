package frc.robot.subystems.Hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);
  
  }


  public void runHopper() {
    io.setTopVelocity(HopperConstants.topMotorTargetVelocity);
    io.setBottomVelocity(HopperConstants.bottomMotorTargetVelocity);
  }

  public void idleHopperSpeed(){
    io.setTopVelocity(HopperConstants.idleTopTargetVelocity);
    io.setBottomVelocity(HopperConstants.idleBottomTargetVelocity);
  }


  public void stop() {
    io.stop();
  }
}