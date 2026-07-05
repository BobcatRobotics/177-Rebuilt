package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper  extends SubsystemBase {

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();


    private HopperState currentState = HopperState.IDLE;

  public Hopper(HopperIO io) {
    this.io = io;
  }
    @Override
  public void periodic() {
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Hopper/inputs", inputs);
  }
    public void stop() {
        io.stop();
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public void setState(HopperState state) {
        currentState = state;
    }

    public HopperState getState() {
        return currentState;
    }
}
