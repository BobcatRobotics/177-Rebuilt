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

    // sysIdRegistry.register("SysIdStateFlywheel", new SysIdModule(
    //     "Shooter/SysIdStateFlywheel",
    //     this,
    //     this::runCharacterization_Flywheel, flyWheelSysIdconfig));
    // sysIdRegistry.register("SysIdStateHood", new SysIdModule(
    //     "Shooter/SysIdStateHood",
    //     this,
    //     this::runCharacterization_Hood, HoodSysIdconfig));

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

    autoPeriodic();

  }

  public void autoPeriodic(){
    Distance distanceToHub = distanceToHub();
    boolean hubInrange = isHubInRange(distanceToHub.getActualDistance(), 5);
    Translation2d[] shotLine = getShotLine(distanceToHub.getActualDistance());
    RobotState.getInstance().hubInrange = hubInrange;
    RobotState.getInstance().shooterUpToSpeed = atSpeed();
    RobotState.getInstance().hubDistance = distanceToHub.getOffsetDistance();
    Logger.recordOutput("Shooter/IsInTarget", hubInrange);
    Logger.recordOutput("Shooter/distanceToHub/actual", distanceToHub.getActualDistance());
    Logger.recordOutput("Shooter/distanceToHub/offset", distanceToHub.getOffsetDistance());
    Logger.recordOutput("Shooter/BallPath", shotLine);
  }

 public Distance distanceToHub() {
    Pose2d robotPose = RobotState.getInstance().robotPose;
    Pose3d hubCoordinate = HubUtil.getMyHubCoordinates(RobotState.getInstance().alliance);
    Translation2d target = hubCoordinate.toPose2d().getTranslation();
    Translation2d robotTranslation = robotPose.getTranslation();
    double distance = robotTranslation.getDistance(target);
    return new Distance(distance, Constants.ShooterConstants.ValuesOfKnownShots.offsetDistanceInMeters);
  }

  public Translation2d[] getShotLine(double distance) {
    Pose2d robotPose = RobotState.getInstance().robotPose;
    Pose2d newPose = robotPose.transformBy(new Transform2d(
        new Translation2d(Units.inchesToMeters(distance), 0),
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
    setShot(desiredState.getCurrentState());
  }

  private void setShot(ShooterState.State state) {
    desiredState.setState(state);
    io.setShot(desiredState);
  }

  private void setVelocity(double dumperLeftSpeed, double dumperRightSpeed, double adjustableHoodPosition) {
    io.setVelocity(dumperLeftSpeed, dumperRightSpeed, adjustableHoodPosition);
  }

  public void dumperLeftSpeed(double dumperLeftSpeed) {
    io.setDumperLeftSpeed(dumperLeftSpeed);
  }

  public void dumperRightSpeed(double dumperRightSpeed) {
    io.setDumperRightSpeed(dumperRightSpeed);
  }

  public void setAdjustableHoodPosition(double adjustableHoodPosition) {
    io.setAdjustableHoodPosition(adjustableHoodPosition);
  }

  public void holdPosition() {
    io.holdPosition();
  }

  public void stop() {
    setIdle();
    io.stop();
  }

  public void stopDumperLeft() {
    io.stopDumperLeft();
  }

  public void stopDumperRight() {
    io.stopDumperRight();

  }

  public void stopAdjustableHood(){
    io.stopAdjustableHood();
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

  // public void runCharacterization_Hood(double output) {
  //   io.runCharacterization_Hood(output);
  // }

  /**
   * Returns the average velocity of the modules in rotations/sec (Phoenix native
   * units).
   */
  // public double getFFCharacterizationVelocity_Hood() {
  //   double output = io.getFFCharacterizationVelocity_Hood();
  //   return output;
  // }


  public SysIdRegistry getRegistry() {
    return sysIdRegistry;
  }


  public void setIdle() {
    RobotState.getInstance().getShooterState().setState(ShooterState.State.IDLE);
    ShooterGoal goal = new ShooterGoal();
    goal.leftDumperSpeed = RobotState.getInstance().getShooterState().getLeftDumperSpeed();
    goal.rightDumperSpeed = RobotState.getInstance().getShooterState().getRightDumperSpeed();
    goal.hoodPosition = RobotState.getInstance().getShooterState().getAdjustableHoodPosition();
    RobotState.getInstance().getShooterState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getShooterState());
  }

  public void manualSpinUp() {
    RobotState.getInstance().getShooterState().setState(ShooterState.State.TARGETING);
    ShooterGoal goal = new ShooterGoal();
    goal.leftDumperSpeed = RobotState.getInstance().getShooterState().getLeftDumperSpeed();
    goal.rightDumperSpeed = RobotState.getInstance().getShooterState().getRightDumperSpeed();
    goal.hoodPosition = RobotState.getInstance().getShooterState().getAdjustableHoodPosition();
    RobotState.getInstance().getShooterState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getShooterState());
  }

  public void manualShootFuel() {
    RobotState.getInstance().getShooterState().setState(ShooterState.State.TARGETING);
    ShooterGoal goal = new ShooterGoal();
    goal.leftDumperSpeed = RobotState.getInstance().getShooterState().getLeftDumperSpeed();
    goal.rightDumperSpeed = RobotState.getInstance().getShooterState().getRightDumperSpeed();
    goal.hoodPosition = RobotState.getInstance().getShooterState().getAdjustableHoodPosition();
    RobotState.getInstance().getShooterState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getShooterState());
  }


  public void spinUp() {
    RobotState.getInstance().getShooterState().setState(ShooterState.State.INTERPOLATING);
    ShooterGoal goal = new ShooterGoal();
    goal.leftDumperSpeed = RobotState.getInstance().getShooterState().getLeftDumperSpeed();
    goal.rightDumperSpeed = RobotState.getInstance().getShooterState().getRightDumperSpeed();
    goal.hoodPosition = RobotState.getInstance().getShooterState().getAdjustableHoodPosition();
    RobotState.getInstance().getShooterState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getShooterState());
  }

  public void shootFuel() {
    RobotState.getInstance().getShooterState().setState(ShooterState.State.INTERPOLATING);
    ShooterGoal goal = new ShooterGoal();
    goal.leftDumperSpeed = RobotState.getInstance().getShooterState().getLeftDumperSpeed();
    goal.rightDumperSpeed = RobotState.getInstance().getShooterState().getRightDumperSpeed();
    goal.hoodPosition = RobotState.getInstance().getShooterState().getAdjustableHoodPosition();
    RobotState.getInstance().getShooterState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getShooterState());
  }

  public void reverseFuel() {
    RobotState.getInstance().getShooterState().setState(ShooterState.State.TARGETING);
    ShooterGoal goal = new ShooterGoal();
    // goal.flywheelSpeed = RobotState.getInstance().getShooterState().getFlywheelSpeed() * -1;
    // goal.hoodSpeed = RobotState.getInstance().getShooterState().getHoodSpeed() * -1;
    goal.leftDumperSpeed = RobotState.getInstance().getShooterState().getLeftDumperSpeed()*-1;
    goal.rightDumperSpeed = RobotState.getInstance().getShooterState().getRightDumperSpeed()*-1;
    goal.hoodPosition = RobotState.getInstance().getShooterState().getAdjustableHoodPosition();
    RobotState.getInstance().getShooterState().setCurrentSetPoints(goal);
    setState(RobotState.getInstance().getShooterState());
  }

  public boolean atSpeed() {
    boolean isAtTolerance = false;
    boolean isDumperLeftWithinTolerance = false;
    boolean isDumperRightWithinTolerance = false;
    boolean isAdjustableHoodWithinTolerance = false;

    double MAIN_SPEED_TOLERANCE = 5; //Using for both dumpers
    double HOOD_POSITION_TOLERANCE = 1;
    isDumperLeftWithinTolerance = Math.abs(getVelocityDumperLeft()
        - RobotState.getInstance().getShooterState().getLeftDumperSpeed()) <= MAIN_SPEED_TOLERANCE;
    isDumperRightWithinTolerance = Math
        .abs(getVelocityOfDumperRight() - RobotState.getInstance().getShooterState().getRightDumperSpeed()) <= MAIN_SPEED_TOLERANCE;
    isAdjustableHoodWithinTolerance = Math.abs(getAdjustableHoodPosition()
        - RobotState.getInstance().getShooterState().getAdjustableHoodPosition()) <= HOOD_POSITION_TOLERANCE;
    if (isDumperLeftWithinTolerance && isDumperRightWithinTolerance && isAdjustableHoodWithinTolerance) {
      isAtTolerance = true;
    }
    Logger.recordOutput("Shooter/isUpToSpeed", isAtTolerance);
    return isAtTolerance;
  }

  public double getVelocityDumperLeft() {
    int count = 0;
    double avg = 0;
    if (inputs.velocityOfDumperLeftUpRPS > 0) {
      avg += inputs.velocityOfDumperLeftUpRPS;
      count += 1;
    }
    if (inputs.velocityOfDumperLeftDownRPS > 0) {
      avg += inputs.velocityOfDumperLeftDownRPS;
      count += 1;
    }

    return avg / count;
  }

  public double getAdjustableHoodPosition() {
    int count = 0;
    double avg = 0;
    if (inputs.positionOfAdjustableHood > 0) {
      avg += inputs.positionOfAdjustableHood;
      count += 1;
    }
    if (inputs.positionOfAdjustableHood > 0) {
      avg += inputs.positionOfAdjustableHood;
      count += 1;
    }

    return avg / count;
  }

  public double getVelocityOfDumperRight() {
    int count = 0;
    double avg = 0;
    if (inputs.velocityOfDumperRightUpRPS > 0) {
      avg += inputs.velocityOfDumperRightUpRPS;
      count += 1;
    }
    if (inputs.velocityOfDumperRightDownRPS > 0) {
      avg += inputs.velocityOfDumperRightDownRPS;
      count += 1;
    }
    // if (inputs.velocityOfMainFlywheelOuterRightRPS > 0) {
    //   avg += inputs.velocityOfMainFlywheelOuterRightRPS;
    //   count += 1;
    // }
    // if (inputs.velocityOfMainFlywheelOuterLeftRPS > 0) {
    //   avg += inputs.velocityOfMainFlywheelOuterLeftRPS;
    //   count += 1;
    // }
    return avg / count;
  }
}