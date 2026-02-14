package frc.robot.subsystems.Shooter;

import org.bobcatrobotics.Hardware.Characterization.SysIdModule;
import org.bobcatrobotics.Hardware.Characterization.SysIdRegistry;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Shooter.ShooterState.State;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private ShooterState desiredState;
  private final SysIdRegistry sysIdRegistry = new SysIdRegistry();

  public Shooter(ShooterIO io) {
    // Configure SysId

    SysIdRoutine.Config flyWheelSysIdconfig = new SysIdRoutine.Config(
        null, // ramp rate
        null, // step voltage
        null, // timeout
        state -> Logger.recordOutput("Shooter/Flywheel/SysIdState", state.toString()));
    SysIdRoutine.Config backspinSysIdconfig = new SysIdRoutine.Config(
        null, // ramp rate
        null, // step voltage
        null, // timeout
        state -> Logger.recordOutput("Shooter/Backspin/SysIdState", state.toString()));
    SysIdRoutine.Config intakeSysIdconfig = new SysIdRoutine.Config(
        null, // ramp rate
        null, // step voltage
        null, // timeout
        state -> Logger.recordOutput("Shooter/Intake/SysIdState", state.toString()));

    sysIdRegistry.register("SysIdStateFlywheel", new SysIdModule(
        "Shooter/SysIdStateFlywheel",
        this,
        this::runCharacterization_Flywheel, flyWheelSysIdconfig));
    sysIdRegistry.register("SysIdStateBackspin", new SysIdModule(
        "Shooter/SysIdStateBackspin",
        this,
        this::runCharacterization_Backspin, backspinSysIdconfig));
    sysIdRegistry.register("SysIdStateIntake", new SysIdModule(
        "Shooter/SysIdStateIntake",
        this,
        this::runCharacterization_Intake, intakeSysIdconfig));

    this.io = io;
  }

  public void applyState() {
    desiredState = new ShooterState();
    desiredState.setState(State.IDLE);
  }

  @Override
  public void periodic() {
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/inputs", inputs);
    Logger.recordOutput("Shooter/State", desiredState.getCurrentState());
  }

  public void setState(ShooterState state) {
    desiredState = state;
    setVelocity(desiredState.getCurrentState());
  }

  private void setVelocity(ShooterState.State state) {
    desiredState.setState(state);
    io.setVelocity(desiredState);
  }

  private void setVelocity(double shooterSpeed, double shooterBackspinSpeedLeft, double shooterBackspinSpeedRight,
      double shooterIntakeSpeed) {
    io.setVelocity(shooterSpeed, shooterBackspinSpeedLeft, shooterBackspinSpeedRight, shooterIntakeSpeed);
  }

  public void setMainWheelSpeed(double shooterFlywheelSpeed) {
    io.setMainWheelSpeed(shooterFlywheelSpeed);
  }

  public void setBackspinSpeedLeft(double shooterBackspinSpeed) {
    io.setBackspinSpeedLeft(shooterBackspinSpeed);
  }

  public void setBackspinSpeedRight(double shooterBackspinSpeed) {
    io.setBackspinSpeedRight(shooterBackspinSpeed);
  }

  public void setIntakeSpeed(double shooterIntakeSpeed) {
    io.setIntakeSpeed(shooterIntakeSpeed);
  }

  public void holdPosition() {
    io.holdPosition();
  }

  public void stop() {
    io.stop();
  }

  public void stopMainWheel() {
    io.stopMainWheel();
  }

  public void stopBackspinWheel() {
    io.stopBackspinWheel();

  }

  public void stopIntakeWheel() {
    io.stopBackspinWheel();
  }

  @Override
  public void simulationPeriodic() {
    io.simulationPeriodic();
  }

  public void runCharacterization_Flywheel(double output) {
    io.runCharacterization_Flywheel(output);
  }

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native
   * units).
   */
  public double getFFCharacterizationVelocity_Flywheel() {
    double output = io.getFFCharacterizationVelocity_Flywheel();
    return output;
  }

  public void runCharacterization_Backspin(double output) {
    io.runCharacterization_Backspin(output);
  }

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native
   * units).
   */
  public double getFFCharacterizationVelocity_Backspin() {
    double output = io.getFFCharacterizationVelocity_Backspin();
    return output;
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