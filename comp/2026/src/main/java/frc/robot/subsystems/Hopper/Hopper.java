package frc.robot.subsystems.Hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

    private final HopperIO io;

    private HopperState hopperState = HopperState.IDLE;

    public Hopper(HopperIO io)
    {
        this.io = io;
    }

    public void setState(HopperState state)
    {
        this.hopperState = state;
    }

    public HopperState getState()
    {
        return this.hopperState;
    }

    @Override
    public void periodic(){
        double targetVelocity = hopperState.getRPS();

        switch(hopperState){
            case IDLE:
                io.stop();
                break;
            default:
                io.setVelocity(targetVelocity);
                break;
        }

    }
    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }
}
