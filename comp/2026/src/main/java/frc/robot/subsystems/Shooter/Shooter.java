package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Rotation;

import org.bobcatrobotics.Hardware.Characterization.SysIdModule;
import org.bobcatrobotics.Hardware.Characterization.SysIdRegistry;
import org.bobcatrobotics.Util.Interpolators.TripleOutputInterpolator;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Shooter.ShooterState.ShooterGoal;
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
    SysIdRoutine.Config HoodSysIdconfig = new SysIdRoutine.Config(
        null, // ramp rate
        null, // step voltage
        null, // timeout
        state -> Logger.recordOutput("Shooter/Hood/SysIdState", state.toString()));
    SysIdRoutine.Config intakeSysIdconfig = new SysIdRoutine.Config(
        null, // ramp rate
        null, // step voltage
        null, // timeout
        state -> Logger.recordOutput("Shooter/Intake/SysIdState", state.toString()));

    sysIdRegistry.register("SysIdStateFlywheel", new SysIdModule(
        "Shooter/SysIdStateFlywheel",
        this,
        this::runCharacterization_Flywheel, flyWheelSysIdconfig));
    sysIdRegistry.register("SysIdStateHood", new SysIdModule(
        "Shooter/SysIdStateHood",
        this,
        this::runCharacterization_Hood, HoodSysIdconfig));
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
    desiredState.update();
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/inputs", inputs);
    Logger.recordOutput("Shooter/State", desiredState.getCurrentState());

    Pose2d robotPose = RobotState.getInstance().robotPose;
    Pose2d newPose = robotPose.transformBy(new Transform2d(
    new Translation2d(Units.inchesToMeters(90), 0),
    new Rotation2d()
));
    Translation2d robotTranslation = robotPose.getTranslation();
    Translation2d targetTranslation =newPose.getTranslation();
    Translation2d[] shotLine = new Translation2d[] {
          robotTranslation,
          targetTranslation
    };

    Logger.recordOutput("Shooter/BallPath", shotLine);
    boolean isTargetAligned =new Translation2d(4.620, 4.040)
                 .getDistance(newPose.getTranslation()) <= Units.inchesToMeters(40);
    Logger.recordOutput("Shooter/IsInTarget", isTargetAligned);
  }

  public void setState(ShooterState state) {
    desiredState = state;
    setVelocity(desiredState.getCurrentState());
  }

  private void setVelocity(ShooterState.State state) {
    desiredState.setState(state);
    io.setVelocity(desiredState);
  }

  private void setVelocity(double shooterSpeed, double shooterHoodSpeedLeft, double shooterHoodSpeedRight,
      double shooterIntakeSpeed) {
    io.setVelocity(shooterSpeed, shooterHoodSpeedLeft, shooterHoodSpeedRight, shooterIntakeSpeed);
  }

  public void setMainWheelSpeed(double shooterFlywheelSpeed) {
    io.setMainWheelSpeed(shooterFlywheelSpeed);
  }

  public void setHoodSpeedLeft(double shooterHoodSpeed) {
    io.setHoodSpeedLeft(shooterHoodSpeed);
  }

  public void setHoodSpeedRight(double shooterHoodSpeed) {
    io.setHoodSpeedRight(shooterHoodSpeed);
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

  public void stopHoodWheel() {
    io.stopHoodWheel();

  }

  public void stopIntakeWheel() {
    io.stopIntakeWheel();
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

  public void runCharacterization_Hood(double output) {
    io.runCharacterization_Hood(output);
  }

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native
   * units).
   */
  public double getFFCharacterizationVelocity_Hood() {
    double output = io.getFFCharacterizationVelocity_Hood();
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


  public void spinUp() {
    RobotState.getInstance().getShooterState().setState(ShooterState.State.TARGETING);
    ShooterGoal goal = new ShooterGoal();
    goal.flywheelSpeed = RobotState.getInstance().getShooterState().getFlywheelSpeed();
    goal.hoodSpeed = RobotState.getInstance().getShooterState().getHoodSpeed();
    goal.intakeSpeed = 0;
    RobotState.getInstance().getShooterState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getShooterState());
  }

  public void shootFuel() {
    RobotState.getInstance().getShooterState().setState(ShooterState.State.TARGETING);
    ShooterGoal goal = new ShooterGoal();
    goal.flywheelSpeed = RobotState.getInstance().getShooterState().getFlywheelSpeed();
    goal.hoodSpeed = RobotState.getInstance().getShooterState().getHoodSpeed();
    goal.intakeSpeed = RobotState.getInstance().getShooterState().getIntakeSpeed();
    RobotState.getInstance().getShooterState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getShooterState());
  }
  public void reverseFuel() {
    RobotState.getInstance().getShooterState().setState(ShooterState.State.TARGETING);
    ShooterGoal goal = new ShooterGoal();
    goal.flywheelSpeed = RobotState.getInstance().getShooterState().getFlywheelSpeed() * -1;
    goal.hoodSpeed = RobotState.getInstance().getShooterState().getHoodSpeed() * -1;
    goal.intakeSpeed = RobotState.getInstance().getShooterState().getIntakeSpeed() * -1;
    RobotState.getInstance().getShooterState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getShooterState());
  }
  
  public boolean atSpeed() {
    boolean isAtTolerance = false;
    double MAIN_SPEED_TOLERANCE = 7;
    double HOOD_SPEED_TOLERANCE = 7;
    double flywheelSpeedVelocity = RobotState.getInstance().getShooterState().getFlywheelSpeed() - MAIN_SPEED_TOLERANCE;
    double hoodSpeedVelocity = RobotState.getInstance().getShooterState().getHoodSpeed() - HOOD_SPEED_TOLERANCE;
    if (io.getVelocityHood() > 30) {
      if (io.getVelocityMainFlywheel() > 40) {
        isAtTolerance = true;
      }
    }
    Logger.recordOutput("Shooter/isUpToSpeed", isAtTolerance);
    return isAtTolerance;
  }
}