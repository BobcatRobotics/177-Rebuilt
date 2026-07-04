package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
  public class IntakeIOInputs {
    public double velocityOfIntakePositionRPS = 0.0;
    public double statorCurrentOfIntakePositionAmps = 0.0;
    public double outputOfIntakePositionVolts = 0.0;
    public double accelerationOfIntakePosition = 0.0;
    public boolean positionConnected = false;
    public double intakePosition = -1;

    public double leftVelocityOfIntakeSpeedRPS = 0.0;
    public double leftStatorCurrentOfIntakeSpeedAmps = 0.0;
    public double leftOutputOfIntakeSpeedVolts = 0.0;
    public double leftAccelerationOfIntakeSpeed = 0.0;
    public boolean leftVelocityMotorConnected = false;

    public double rightVelocityOfIntakeSpeedRPS = 0.0;
    public double rightStatorCurrentOfIntakeSpeedAmps = 0.0;
    public double rightOutputOfIntakeSpeedVolts = 0.0;
    public double rightAccelerationOfIntakeSpeed = 0.0;
    public boolean rightVelocityMotorConnected = false;
  }

    public default void updateInputs(IntakeIOInputs inputs) {
    }

    public default void stop() {
    }
}
