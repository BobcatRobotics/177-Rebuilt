package frc.robot.subsystems.Shooter;

import org.bobcatrobotics.GameSpecific.Rebuilt.HubUtil;
import org.bobcatrobotics.Hardware.Characterization.SysIdModule;
import org.bobcatrobotics.Hardware.Characterization.SysIdRegistry;
import org.bobcatrobotics.Util.Interpolators.TripleOutputInterpolator;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private ShooterState desiredState;
  private final SysIdRegistry sysIdRegistry = new SysIdRegistry();

  private TripleOutputInterpolator interpolator = new TripleOutputInterpolator(
      Constants.ShooterConstants.ValuesOfKnownShots.distance,
      Constants.ShooterConstants.ValuesOfKnownShots.carwashSpeed,
      Constants.ShooterConstants.ValuesOfKnownShots.hoodSpeed,
      Constants.ShooterConstants.ValuesOfKnownShots.mainFlyWheelSpeed,
      true);

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

    double distanceToHub = distanceToHub();
    boolean hubInrange = isHubInRange(distanceToHub, 20);
    Translation2d[] shotLine = getShotLine(distanceToHub);
    ShooterState.ShooterGoal shooterSpeeds = getShooterSpeeds(distanceToHub);
    RobotState.getInstance().hubInrange = hubInrange;
    RobotState.getInstance().shooterUpToSpeed = atSpeed();
    Logger.recordOutput("Shooter/IsInTarget", hubInrange);
    Logger.recordOutput("Shooter/distanceToHub", distanceToHub);
    Logger.recordOutput("Shooter/BallPath", shotLine);
    Logger.recordOutput("Shooter/Speeds/flywheel", shooterSpeeds.flywheelSpeed);
    Logger.recordOutput("Shooter/Speeds/carwash", shooterSpeeds.intakeSpeed);
    Logger.recordOutput("Shooter/Speeds/hood", shooterSpeeds.hoodSpeed);

  }

  public ShooterState.ShooterGoal getShooterSpeeds(double distance) {
    ShooterState.ShooterGoal goal = new ShooterGoal();
    double angle = 90 - 24.7;
    double partA = 3 / Math.PI;
    double numerator = 32.2 * (distance * distance);
    double denominator = 2 * Math.cos(angle) * (distance * Math.tan(angle) - 4.657);
    goal.flywheelSpeed = partA * Math.sqrt(numerator / denominator);
    goal.intakeSpeed = 2 * goal.flywheelSpeed;
    goal.hoodSpeed = 0.4 * goal.flywheelSpeed;
    return goal;
  }

  public double getTrajectoryShotHeight(ShooterState.ShooterGoal speeds) {
    double theta = Math.toRadians(90 - 24.7);
    double g = 32.2; // ft/s^2
    double carwashRadius = 1;
    double hoodRadius = 1;
    double flywheelRadius = 2;
    double velocity = carwashRadius * speeds.intakeSpeed + flywheelRadius * speeds.flywheelSpeed
        + hoodRadius * speeds.hoodSpeed;
    double height = (velocity * velocity * Math.pow(Math.sin(theta), 2)) / (2 * g);
    double shooterExitHeight = 17.2;
    double totalHeight = shooterExitHeight + height;
    return totalHeight;
  }

  public double distanceToHub() {
    Pose2d robotPose = RobotState.getInstance().robotPose;
    Pose3d hubCoordinate = HubUtil.getMyHubCoordinates(RobotState.getInstance().alliance);
    Translation2d target = hubCoordinate.toPose2d().getTranslation();
    Translation2d robotTranslation = robotPose.getTranslation();
    double distance = robotTranslation.getDistance(target);
    return distance;
  }

  public Translation2d[] getShotLine(double distance) {
    Pose2d robotPose = RobotState.getInstance().robotPose;
    Pose2d newPose = robotPose.transformBy(new Transform2d(
        new Translation2d(Units.inchesToMeters(15) + distance, 0),
        new Rotation2d()));
    Translation2d robotTranslation = robotPose.getTranslation();
    Translation2d targetTranslation = newPose.getTranslation();
    Translation2d[] shotLine = new Translation2d[] {
        robotTranslation,
        targetTranslation
    };
    return shotLine;
  }

  public boolean isHubInRange(double distance, double radius) {

    Pose2d robotPose = RobotState.getInstance().robotPose;
    Pose2d newPose = robotPose.transformBy(new Transform2d(
        new Translation2d(Units.inchesToMeters(15) + distance, 0),
        new Rotation2d()));
    Pose3d hubCoords = HubUtil.getMyHubCoordinates(RobotState.getInstance().alliance);
    boolean hubInrange = hubCoords.toPose2d().getTranslation()
        .getDistance(newPose.getTranslation()) <= Units.inchesToMeters(radius);
    return hubInrange;
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
    goal.flywheelSpeed = Constants.ShooterConstants.targetFlywheelSpeedRPS;
    goal.hoodSpeed = Constants.ShooterConstants.targetHoodSpeedRPS;
    goal.intakeSpeed = 0;
    RobotState.getInstance().getShooterState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getShooterState());
  }

  public void shootFuel() {
    RobotState.getInstance().getShooterState().setState(ShooterState.State.TARGETING);
    ShooterGoal goal = new ShooterGoal();
    goal.flywheelSpeed = Constants.ShooterConstants.targetFlywheelSpeedRPS;
    goal.hoodSpeed = Constants.ShooterConstants.targetHoodSpeedRPS;
    goal.intakeSpeed = Constants.ShooterConstants.targetIntakeSpeedRPS;
    RobotState.getInstance().getShooterState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getShooterState());
  }

  public void shootFuel(DoubleSupplier flywheelrps, DoubleSupplier hoodrps, DoubleSupplier carwashrps) {
    RobotState.getInstance().getShooterState().setState(ShooterState.State.TARGETING);
    ShooterGoal goal = new ShooterGoal();
    goal.flywheelSpeed = flywheelrps.getAsDouble();
    goal.hoodSpeed = hoodrps.getAsDouble();
    goal.intakeSpeed = carwashrps.getAsDouble();
    RobotState.getInstance().getShooterState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getShooterState());
  }

  public void spinUp(double distanceToHub) {
    RobotState.getInstance().getShooterState().setState(ShooterState.State.TARGETING);
    ShooterGoal goal = new ShooterGoal();
    goal.flywheelSpeed = interpolator.getAsList(distanceToHub).get(0);
    ;
    goal.hoodSpeed = interpolator.getAsList(distanceToHub).get(1);
    goal.intakeSpeed = 0;
    RobotState.getInstance().getShooterState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getShooterState());
  }

  public void shootFuel(double distanceToHub) {
    RobotState.getInstance().getShooterState().setState(ShooterState.State.TARGETING);
    ShooterGoal goal = new ShooterGoal();
    goal.flywheelSpeed = interpolator.getAsList(distanceToHub).get(0);
    ;
    goal.hoodSpeed = interpolator.getAsList(distanceToHub).get(1);
    goal.intakeSpeed = interpolator.getAsList(distanceToHub).get(2);
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
    boolean isMainFlywheelWithinTolerance = false;
    boolean isHoodWheelWithinTolerance = false;

    double MAIN_SPEED_TOLERANCE = 5;
    double HOOD_SPEED_TOLERANCE = 2;
    isMainFlywheelWithinTolerance = Math.abs(getVelocityMainFlywheel()
        - RobotState.getInstance().getShooterState().getFlywheelSpeed()) <= MAIN_SPEED_TOLERANCE;
    isHoodWheelWithinTolerance = Math
        .abs(getVelocityHood() - RobotState.getInstance().getShooterState().getHoodSpeed()) <= HOOD_SPEED_TOLERANCE;
    if (isMainFlywheelWithinTolerance && isHoodWheelWithinTolerance) {
      isAtTolerance = true;
    }
    Logger.recordOutput("Shooter/isUpToSpeed", isAtTolerance);
    return isAtTolerance;
  }

  public double getVelocityHood() {
    int count = 0;
    double avg = 0;
    if (inputs.velocityOfHoodWheelMotorLeftRPS > 0) {
      avg += inputs.velocityOfHoodWheelMotorLeftRPS;
      count += 1;
    }
    if (inputs.velocityOfHoodWheelMotorRightRPS > 0) {
      avg += inputs.velocityOfHoodWheelMotorRightRPS;
      count += 1;
    }

    return avg / count;
  }

  public double getVelocityMainFlywheel() {
    int count = 0;
    double avg = 0;
    if (inputs.velocityOfMainFlywheelLeftRPS > 0) {
      avg += inputs.velocityOfMainFlywheelLeftRPS;
      count += 1;
    }
    if (inputs.velocityOfMainFlywheelRightRPS > 0) {
      avg += inputs.velocityOfMainFlywheelRightRPS;
      count += 1;
    }
    if (inputs.velocityOfMainFlywheelOuterRightRPS > 0) {
      avg += inputs.velocityOfMainFlywheelOuterRightRPS;
      count += 1;
    }
    if (inputs.velocityOfMainFlywheelOuterLeftRPS > 0) {
      avg += inputs.velocityOfMainFlywheelOuterLeftRPS;
      count += 1;
    }
    return avg / count;
  }
}