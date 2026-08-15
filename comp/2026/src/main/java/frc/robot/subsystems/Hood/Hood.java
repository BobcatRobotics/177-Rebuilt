package frc.robot.subsystems.Hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {

    private final HoodIO io;

    private HoodState state = HoodState.IDLE;

    public Hood(HoodIO io) {
        this.io = io;
    }

    public void setState(HoodState state) {
        this.state = state;
    }

    public HoodState getState() {
        return state;
    }

    public boolean atTargetAngle() {

        return io.atTargetAngle(state.getAngleDegrees());
    }

    @Override
    public void periodic() {

        switch(state){
            case IDLE:
                io.stop();
                break;
            default:
                io.setAngle(state.getAngleDegrees());
                break;
        }
    }

    @Override
    public void simulationPeriodic() {

        io.simulationPeriodic();
    }
}