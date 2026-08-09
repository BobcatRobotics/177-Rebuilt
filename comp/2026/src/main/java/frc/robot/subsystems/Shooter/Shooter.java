package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;

    private ShooterState state = ShooterState.IDLE;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public ShooterState getState() {
        return state;
    }

    public double getVelocityRPS() {
        return io.getVelocityRPS();
    }

    public boolean atTargetSpeed() {

        double target = state.getRPS();
        double actual = io.getVelocityRPS();

        return Math.abs(target - actual) < 2.0;
    }

    @Override
    public void periodic() {

        io.setVelocity(
            state.getRPS()
        );
    }
}