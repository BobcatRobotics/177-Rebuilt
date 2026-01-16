package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs =
      new ShooterIOInputsAutoLogged();

  private ShooterState desiredState = ShooterState.IDLE;
  private boolean manualMode = false;

  private double manualTopVolts = 0.0;
  private double manualBottomVolts = 0.0;
  private double manualKickerVolts = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/State", desiredState.name());
    Logger.recordOutput("Shooter/ManualMode", manualMode);

    if (manualMode) {
      io.setTopVoltage(manualTopVolts);
      io.setBottomVoltage(manualBottomVolts);
      io.setKickerVoltage(manualKickerVolts);
    } else {
      io.setTopVelocity(desiredState.topRPM);
      io.setBottomVelocity(desiredState.bottomRPM);
      io.setKickerVelocity(desiredState.kickerRPM);
    }
  }

  public void setManualVoltage(double top, double bottom, double kicker) {
        manualMode = true;
    manualTopVolts = top;
    manualBottomVolts = bottom;
    manualKickerVolts = kicker;
  }

  public void exitManualMode() {
     manualMode = false;
  }

  public void setState(ShooterState state) {
    manualMode = false;
    desiredState = state;
  }

  public boolean atSetpoint() {
    return Math.abs(inputs.topVelocityRPM - desiredState.topRPM) < 100
        && Math.abs(inputs.bottomVelocityRPM - desiredState.bottomRPM) < 100 && Math.abs(inputs.kickerVelocityRPM - desiredState.kickerRPM) < 100;
  }

  public void stop(){
    io.stop();
  }
}