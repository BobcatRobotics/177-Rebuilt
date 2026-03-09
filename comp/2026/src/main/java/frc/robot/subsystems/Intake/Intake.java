package frc.robot.subsystems.Intake;

import org.bobcatrobotics.Hardware.Characterization.SysIdModule;
import org.bobcatrobotics.Hardware.Characterization.SysIdRegistry;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.subsystems.Intake.IntakeState.IntakeGoal;
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
        this::runCharacterization_IntakeVelocity, IntakeSysIdconfig));

    this.io = io;
  }

  public void applyState() {
    desiredState = new IntakeState();
    desiredState.setState(State.IDLE);
  }

  @Override
  public void periodic() {
    desiredState.update();
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Intake/inputs", inputs);
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

   public void retractIntakeManually(){
    io.retractIntake();
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

  public void runCharacterization_IntakeVelocity(double output) {
    io.runCharacterization_IntakeVelocity(output);
  }

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native
   * units).
   */
  public double getFFCharacterizationVelocity_Intake() {
    double output = io.getFFCharacterizationVelocity_Intake();
    return output;
  }

  public double getFFCharacterizationPosition_Intake() {
    double output = io.getFFCharacterizationPosition_Intake();
    return output;
  }

  public SysIdRegistry getRegistry() {
    return sysIdRegistry;
  }

  public void grabBalls() {
    RobotState.getInstance().getIntakeState().setState(IntakeState.State.MANUAL);
    IntakeGoal goal = new IntakeGoal();
    goal.speed = 40;
    goal.position = 6;
    RobotState.getInstance().getIntakeState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getIntakeState());
  }

  public void resetEncoder(){
    io.resetEncoder();
  }

  public void retractIntake() {
    RobotState.getInstance().getIntakeState().setState(IntakeState.State.MANUAL);
    IntakeGoal goal = new IntakeGoal();
    goal.speed = 0;
    goal.position = 0;
    RobotState.getInstance().getIntakeState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getIntakeState());
  }

  public double getPosition(){
    return inputs.intakePosition;
  }


  public Command retractAndStop(){
    return new RunCommand(() -> {
                        retractIntakeManually();
                }).onlyWhile(()->!DriverStation.isDisabled()).andThen(new InstantCommand(()->stop()));
  }
}