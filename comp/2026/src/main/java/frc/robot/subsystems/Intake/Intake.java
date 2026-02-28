package frc.robot.subsystems.Intake;

import org.bobcatrobotics.Hardware.Characterization.SysIdModule;
import org.bobcatrobotics.Hardware.Characterization.SysIdRegistry;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Intake.IntakeState.State;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private IntakeState desiredState;
  private final SysIdRegistry sysIdRegistry = new SysIdRegistry();

  public Intake(IntakeIO io) {
    // Configure SysId

    SysIdRoutine.Config IntakeSysIdconfig = new SysIdRoutine.Config(
        null, // ramp rate
        null, // step voltage
        null, // timeout
        state -> Logger.recordOutput("Intake/SysIdState", state.toString()));

    sysIdRegistry.register("SysIdStateIntake", new SysIdModule(
        "Intake/SysIdStateIntake",
        this,
        this::runCharacterization_Intake, IntakeSysIdconfig));

    this.io = io;
  }

  public void applyState() {
    desiredState = new IntakeState();
    desiredState.setState(State.IDLE);
  }

  @Override
  public void periodic() {
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/State", desiredState.getCurrentState());
  }

  public void setState(IntakeState state) {
    desiredState = state;
    setVelocity(desiredState.getCurrentState());
    setPosition(desiredState.getCurrentState());
  }

  public void setVelocity(IntakeState.State state) {
    desiredState.setState(state);
    io.setVelocity(desiredState);
  }

  public void setVelocity(double intakeRollerVelocity) {
    io.setVelocity(intakeRollerVelocity);
  }

  public void setPosition(IntakeState.State state) {
    desiredState.setState(state);
    io.setPosition(desiredState);
  }

  public void setPosition(double intakePosition) {
    io.setPosition(intakePosition);
  }

  public void holdPosition() {
    io.holdPosition();
  }

  public void stop() {
    io.stop();
  }

  public void stopRollerWheel() {
    io.stopRollerWheel();
  }

  public void stopPivot() {
    io.stopPivotMotor();
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  public void runCharacterization_Intake(double output) {
    io.runCharacterization_Intake(output);
  }

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native
   * units).
   */
  public double getFFCharacterizationVelocity_Intake() {
    double output = io.getFFCharacterizationVelocity_Intake();
    return output;
  }

  public SysIdRegistry getRegistry() {
    return sysIdRegistry;
  }
}