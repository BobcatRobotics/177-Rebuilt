package frc.robot.subsystems.hopper;

import org.bobcatrobotics.Framework.StateMachine.SubsystemStateMachine;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper  extends SubsystemBase {

  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();


    private final SubsystemStateMachine<HopperState> hopperStateMachine =
            new SubsystemStateMachine<>(HopperState.IDLE);


  public Hopper(HopperIO io) {
    this.io = io;
  }
    @Override
  public void periodic() {

    hopperStateMachine.periodic();

    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Hopper/inputs", inputs);

            switch (hopperStateMachine.getState()) {
            case IDLE:
                io.stop();
                break;

            case SPINUP:
                io.setVelocity(hopperStateMachine.getState());
                break;

            case INTAKE:
                io.setVelocity(hopperStateMachine.getState());
                break;

            case OUTTAKE:
                io.setVelocity(hopperStateMachine.getState());
                break;
        }

  }
    public void stop() {
        io.stop();
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public void setState(HopperState state) {
        hopperStateMachine.setState(state);
    }

    public HopperState getState() {
        return hopperStateMachine.getState();
    }

    public boolean inState(HopperState state) {
        return hopperStateMachine.isInState(state);
    }
}
