package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;

    private ShooterState state = ShooterState.OFF;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public ShooterState getState() {
        return state;
    }

    public boolean atTargetSpeed() {
        return io.atTargetSpeed(state.getFlywheelRPS());
    }

    public double getVelocityRPS() {
        return io.getFlywheelVelocityRPS();
    }

    @Override
    public void periodic() {
        io.setFlywheelVelocity(state.getFlywheelRPS());
    }

    @Override
    public void simulationPeriodic() {

        io.simulationPeriodic();
    }
}