package frc.robot.subsystems.Carwash;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Carwash extends SubsystemBase {

    private final CarwashIO io;

    private CarwashState state = CarwashState.IDLE;

    public Carwash(CarwashIO io) {

        this.io = io;
    }

    public void setState(CarwashState state) {

        this.state = state;
    }

    public CarwashState getState() {

        return state;
    }

    /*
     * This is intentionally NOT part of CarwashState.
     *
     * An enum is good for predefined states.
     * Manual control requires a runtime value.
     */

    public void setManualSpeed(double rps) {

        io.setVelocity(rps);
    }

    public double getVelocityRPS() {

        return io.getVelocityRPS();
    }


    public boolean atTargetSpeed() {

        return io.atTargetSpeed(state.getRPS()
        );
    }

    @Override
    public void periodic() {
        io.setVelocity(state.getRPS());
    }

   public void stop() {
        io.stop();
        setState(CarwashState.IDLE);
    }


    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }
}