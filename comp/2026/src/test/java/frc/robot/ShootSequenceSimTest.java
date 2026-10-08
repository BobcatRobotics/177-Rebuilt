package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Carwash.CarwashSim;
import frc.robot.subsystems.Shooter.ShooterRealDrum;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Runs the real robot code in simulation, holds the driver's left trigger (auto-feed shoot) and
 * checks the CT States flicker symptoms: once feeding starts it must not stop, and the hood must
 * not be commanded back to 0 mid-volley.
 *
 * <p>Phoenix 6 simulated motors run in real time, so this test takes real seconds to run.
 */
class ShootSequenceSimTest {
  private static final double LOOP_SECONDS = 0.02;

  private static RobotContainer container;
  private static XboxControllerSim driver;

  /** One sample per robot loop. */
  private record Sample(
      double time,
      boolean feeding,
      double commandedHood,
      boolean atSpeed,
      boolean defaultsIdle,
      double leftDrumRps,
      double rightDrumRps,
      double drumGoalRps,
      double carwashCommandRps) {}

  @BeforeAll
  static void setUp() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();

    container = new RobotContainer();
    driver = new XboxControllerSim(0);
    RobotState.getInstance().alliance = Alliance.Blue;
    // About 47 in from the hub face, where the shot table's hood angle is non-zero
    container.drive.setPose(new Pose2d(2.5, 4.03, Rotation2d.kZero));

    // Let the simulated motor controllers boot and see the enable
    Timer.delay(0.2);
    // Idle like a real robot after enable, so default commands are already scheduled before any
    // button is pressed (in the old code, command run order depended on this)
    run(0.5);
  }

  private static List<Sample> run(double seconds) {
    List<Sample> samples = new ArrayList<>();
    int loops = (int) Math.round(seconds / LOOP_SECONDS);
    for (int i = 0; i < loops; i++) {
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
      CommandScheduler.getInstance().run(); // also runs subsystem simulationPeriodic()
      Timer.delay(LOOP_SECONDS);

      var scheduler = CommandScheduler.getInstance();
      boolean defaultsIdle =
          scheduler.requiring(container.m_Shooter) == container.m_Shooter.getDefaultCommand()
              || scheduler.requiring(container.m_Carwash) == container.m_Carwash.getDefaultCommand();
      samples.add(
          new Sample(
              i * LOOP_SECONDS,
              RobotState.getInstance().getCarwashState().getIntakeSpeed() > 0,
              container.m_Shooter.getCommandedHoodPosition(),
              container.m_Shooter.atSpeed(),
              defaultsIdle,
              container.m_Shooter.getVelocityDumperLeft(),
              container.m_Shooter.getVelocityOfDumperRight(),
              RobotState.getInstance().getShooterState().getLeftDumperSpeed(),
              CarwashSim.simCommandedVelocityRps));
    }
    return samples;
  }

  @Test
  void leftTriggerFeedsContinuouslyWithoutHoodFlicker() {
    // Heavy full-hopper load: big enough that the drum dips past the 5 RPS atSpeed() tolerance,
    // which is what triggered the spin-up/shoot flip-flop at CT States
    ShooterRealDrum.simVelocityLossPerBallRPS = 8.0;
    driver.setLeftTriggerAxis(1.0);
    List<Sample> held = run(6.0);
    driver.setLeftTriggerAxis(0.0);
    run(0.5);

    // Drum trace every 0.25 s, to sanity-check the sim model
    for (int i = 0; i < held.size(); i += 12) {
      Sample s = held.get(i);
      System.out.printf(
          "t=%.2f drumL=%.1f drumR=%.1f goal=%.1f atSpeed=%b feeding=%b hoodCmd=%.2f carwashCmd=%.1f%n",
          s.time(), s.leftDrumRps(), s.rightDrumRps(), s.drumGoalRps(), s.atSpeed(), s.feeding(),
          s.commandedHood(), s.carwashCommandRps());
    }

    int firstFeed = -1;
    for (int i = 0; i < held.size(); i++) {
      if (held.get(i).feeding()) {
        firstFeed = i;
        break;
      }
    }
    assertTrue(firstFeed >= 0, "Never started feeding within 6 s of holding left trigger");

    List<Sample> volley = held.subList(firstFeed, held.size());
    long dipsBelowTolerance = volley.stream().filter(s -> !s.atSpeed()).count();
    assertTrue(
        dipsBelowTolerance > 0,
        "Simulated ball load never dipped the drum out of tolerance, so this test can't detect"
            + " the flip-flop; raise ShooterRealDrum.simVelocityLossPerBallRPS");

    long feedStops = volley.stream().filter(s -> !s.feeding()).count();
    long hoodAtZero = volley.stream().filter(s -> s.commandedHood() == 0.0).count();
    System.out.printf(
        "Feeding started at %.2f s; loops in volley: %d, drum out of tolerance: %d,"
            + " feed stopped: %d, hood commanded to 0: %d%n",
        held.get(firstFeed).time(), volley.size(), dipsBelowTolerance, feedStops, hoodAtZero);

    assertTrue(feedStops == 0, "Feeding stopped " + feedStops + " loops after it started");
    assertTrue(hoodAtZero == 0, "Hood was commanded to 0 on " + hoodAtZero + " loops mid-volley");
    assertFalse(
        held.stream().skip(1).anyMatch(Sample::defaultsIdle),
        "A shooter/carwash default command ran while the shoot button was held");
  }

  /**
   * Driver RB (spin up) + LB (feed), the way the driver shot at CT States. Teleports the robot
   * mid-volley to fake a vision pose jump: with the dashboard lock on, the drum and hood goals must
   * not move; with it off, they must.
   */
  @Test
  void shotDistanceLockHoldsDrumAndHoodThroughPoseJump() {
    double[] lockedChange = poseJumpDuringFeed(true);
    double[] unlockedChange = poseJumpDuringFeed(false);
    System.out.printf(
        "Pose jump mid-volley: lock on -> drum goal moved %.2f RPS, hood goal %.3f;"
            + " lock off -> drum %.2f RPS, hood %.3f%n",
        lockedChange[0], lockedChange[1], unlockedChange[0], unlockedChange[1]);

    assertTrue(
        unlockedChange[0] > 0.5 || unlockedChange[1] > 0.05,
        "With the lock off the pose jump didn't change the shot, so this test can't detect the lock");
    assertTrue(lockedChange[0] == 0.0, "Drum goal moved " + lockedChange[0] + " RPS while locked");
    assertTrue(lockedChange[1] == 0.0, "Hood goal moved " + lockedChange[1] + " while locked");
  }

  /** Returns {|drum goal change|, |hood goal change|} across a pose jump while feeding. */
  private static double[] poseJumpDuringFeed(boolean lock) {
    container.lockShotDistanceWhileFeeding.set(lock);
    container.lockShotDistanceWhileFeeding.periodic();
    container.drive.setPose(new Pose2d(2.5, 4.03, Rotation2d.kZero));

    driver.setRightBumperButton(true);
    run(2.0);
    driver.setLeftBumperButton(true);
    run(0.5);
    var shot = RobotState.getInstance().getShooterState();
    double drumBefore = shot.getLeftDumperSpeed();
    double hoodBefore = shot.getAdjustableHoodPosition();

    // About 1 m farther from the hub, as if a bad vision frame moved the pose
    container.drive.setPose(new Pose2d(3.5, 4.03, Rotation2d.kZero));
    run(0.5);
    double drumAfter = shot.getLeftDumperSpeed();
    double hoodAfter = shot.getAdjustableHoodPosition();

    driver.setLeftBumperButton(false);
    driver.setRightBumperButton(false);
    run(0.5);
    container.lockShotDistanceWhileFeeding.set(false);
    container.lockShotDistanceWhileFeeding.periodic();
    return new double[] {Math.abs(drumAfter - drumBefore), Math.abs(hoodAfter - hoodBefore)};
  }
}
