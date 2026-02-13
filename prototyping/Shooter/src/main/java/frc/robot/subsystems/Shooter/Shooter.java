package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Shooter.ShooterState.State;
import frc.robot.subsystems.Shooter.Modules.SysIdModule;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private ShooterState desiredState;
  private  SysIdModule sysIdFlywheel;
  private  SysIdRoutine sysIdBackspin;
  private  SysIdRoutine sysIdIntake;

  public Shooter(ShooterIO io) {
            // Configure SysId
      sysIdFlywheel = new SysIdModule(
              "Shooter/SysIdStateFlywheel",
              this,
              this::runCharacterization_Flywheel
      );

    // sysIdFlywheel = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,
    //         null,
    //         null,
    //         (state) -> Logger.recordOutput("Shooter/SysIdStateFlywheel", state.toString())),
    //     new SysIdRoutine.Mechanism(
    //         (voltage) -> runCharacterization_Flywheel(voltage.in(Volts)), null, this));
    // sysIdBackspin = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,
    //         null,
    //         null,
    //         (state) -> Logger.recordOutput("Shooter/SysIdStateBackspin", state.toString())),
    //     new SysIdRoutine.Mechanism(
    //         (voltage) -> runCharacterization_Backspin(voltage.in(Volts)), null, this));
    //   sysIdIntake= new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,
    //         null,
    //         null,
    //         (state) -> Logger.recordOutput("Shooter/SysIdStateIntake", state.toString())),
    //     new SysIdRoutine.Mechanism(
    //         (voltage) -> runCharacterization_Flywheel(voltage.in(Volts)), null, this));

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

    /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistaticFlywheel(SysIdRoutine.Direction direction) {
    return Commands.run(() -> runCharacterization_Flywheel(0.0))
        .withTimeout(1.0)
        .andThen(sysIdFlywheel.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamicFlywheel(SysIdRoutine.Direction direction) {
    return Commands.run(() -> runCharacterization_Flywheel(0.0)).withTimeout(1.0).andThen(sysIdFlywheel.dynamic(direction));
  }

      /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistaticBackspin(SysIdRoutine.Direction direction) {
    return Commands.run(() -> runCharacterization_Backspin(0.0))
        .withTimeout(1.0)
        .andThen(sysIdBackspin.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamicBackspin(SysIdRoutine.Direction direction) {
    return Commands.run(() -> runCharacterization_Backspin(0.0)).withTimeout(1.0).andThen(sysIdBackspin.dynamic(direction));
  }


      /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistaticIntake(SysIdRoutine.Direction direction) {
    return Commands.run(() -> runCharacterization_Intake(0.0))
        .withTimeout(1.0)
        .andThen(sysIdIntake.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamicIntake(SysIdRoutine.Direction direction) {
    return Commands.run(() -> runCharacterization_Intake(0.0)).withTimeout(1.0).andThen(sysIdIntake.dynamic(direction));
  }
}