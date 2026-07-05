package frc.robot.subsystems.carwash;

import org.littletonrobotics.junction.AutoLog;

public interface CarwashIO {
    @AutoLog
    class CarwashIOInputs {
        public double velocityOfIntakeRPS = 0;
        public double accelerationOfIntake = 0;
        public double statorCurrentOfIntakeAmps = 0;
        public boolean shooterIntakeMotorConnected = false;
        public double outputOfIntakeVolts = 0;
        public double torqueCurrentCarwashAmps = 0.0;
    }

    public default void updateInputs(CarwashIOInputs inputs) {
    }

    public default void stop() {
    }

    public default void setState(CarwashState state) {
    }

    public default void simulationPeriodic() {

    }

    public default void periodic() {

    }
}
