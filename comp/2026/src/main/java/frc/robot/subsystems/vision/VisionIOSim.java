// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static frc.robot.subsystems.vision.VisionConstants.cameraConstants;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOSim implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;
  private final String name;
  private Rotation2d allianceRotation;


   /**
   * Field2d object containing the simulated raycast path.
   *
   * <p>
   * The path is:
   *
   * <pre>
   * robot -> tag1 -> robot -> tag2 -> robot -> ...
   * </pre>
   */
  private FieldObject2d raycasts;
  /**
   * Deterministic noise source.
   */
  private Random noise;
  LimelightSimSettings settings ;



  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name             The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for
   *                         MegaTag 2.
   */
  public VisionIOSim(String name, Supplier<Rotation2d> rotationSupplier) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    // table.getEntry("imumode_set").setDouble(0);
    table.getEntry("pipeline").setNumber(1);
    this.name = name;
    //LimelightHelpers.SetIMUMode(name, 0);

    allianceRotation = new Rotation2d();
    allianceRotation = rotationSupplier.get();
        // RobotState.getInstance().alliance == Alliance.Red
        // ? Rotation2d.fromDegrees(rotationSupplier.get().getDegrees() + 180)
        // : rotationSupplier.get();

    LimelightHelpers.SetRobotOrientation(name, allianceRotation.getDegrees(), 0, 0, 0, 0, 0);
    this.rotationSupplier = rotationSupplier;
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});

    // limelight is configured
    settings = new LimelightSimSettings();
    this.noise = new Random(settings.randomSeed);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last
    // 250ms
    inputs.connected = ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation = new TargetObservation(
        Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    allianceRotation = rotationSupplier.get();
        // RobotState.getInstance().alliance == Alliance.Red
        // ? Rotation2d.fromDegrees(rotationSupplier.get().getDegrees() + 180)
        // : rotationSupplier.get();

    inputs.externalAngle = allianceRotation.getDegrees();

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] { allianceRotation.getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0 });
    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight
    LimelightHelpers.SetRobotOrientation(name, allianceRotation.getDegrees(), 0, 0, 0, 0, 0);

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) {
        continue;
      }
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, zeroed because the pose is already disambiguated
              0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_2));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }

    update(RobotState.getInstance().robotPose);
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }

  /**
   * Transform from the robot's origin to the camera's lens.
   */
  private Transform3d robotToCamera = new Transform3d();
  /**
   * AprilTag field layout used to source simulated targets.
   */
  private AprilTagFieldLayout fieldLayout;
 
  /**
   * Update the simulated {@link Limelight} with the robot's current pose, projecting visible AprilTags and publishing the resulting NetworkTables data. Call this periodically, typically from
   * {@code Robot#simulationPeriodic()}.
   * <p>
   * No-op if not running in simulation.
   *
   * @param robotPoseMeters Ground-truth robot pose, in meters.
   */
  public void update(Pose2d robotPoseMeters)
  {
    update(new Pose3d(robotPoseMeters));
  }
  /**
   * Update the simulated {@link Limelight} with the robot's current pose, projecting visible AprilTags and publishing the resulting NetworkTables data. Call this periodically, typically from
   * {@code Robot#simulationPeriodic()}.
   * <p>
   * No-op if not running in simulation.
   *
   * @param robotPose Ground-truth robot pose, in meters.
   */
  public void update(Pose3d robotCurrentPose){

    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);


    Pose3d cameraPose = robotCurrentPose.plus(robotToCamera);

    List<TagObservation> visible = new ArrayList<>();

    for (AprilTag tag : fieldLayout.getTags())
    {
      project(cameraPose, robotCurrentPose, tag).ifPresent(visible::add);
    }

    visible.sort((a, b) -> Double.compare(b.ta, a.ta));

    drawRaycasts(robotCurrentPose, visible);

  }

  /**
   * Project a field {@link AprilTag} into the simulated camera, returning an observation if it is within the configured field of view, range and incidence angle.
   */
  private Optional<TagObservation> project(Pose3d cameraPose, Pose3d robotPose, AprilTag tag)
  {
    Pose3d tagPose = tag.pose;

    Transform3d camToTag = new Transform3d(cameraPose, tagPose);

    double x = camToTag.getX();
    double y = camToTag.getY();
    double z = camToTag.getZ();

    if (x <= 0.02)
    {
      return Optional.empty();
    }

    double distance = camToTag.getTranslation().getNorm();

    

    if (distance > settings.maxDetectionRangeMeters)
    {
      return Optional.empty();
    }

    double halfHFovDeg = settings.horizontalFOV.getDegrees() / 2.0;

    double halfVFovDeg = settings.verticalFOV.getDegrees() / 2.0;

    // tx positive = target right of crosshair
    // ty positive = target below crosshair
    // Limelight convention.
    double yawDeg = Math.toDegrees(Math.atan2(-y, x));

    double pitchDeg = Math.toDegrees(Math.atan2(-z, Math.hypot(x, y)));

    if (Math.abs(yawDeg) > halfHFovDeg || Math.abs(pitchDeg) > halfVFovDeg)
    {
      return Optional.empty();
    }

    Translation3d tagNormal = new Translation3d(1, 0, 0).rotateBy(tagPose.getRotation());

    Translation3d tagToCam = cameraPose.getTranslation().minus(tagPose.getTranslation());

    double dot = tagNormal.getX() * tagToCam.getX() + tagNormal.getY() * tagToCam.getY() + tagNormal.getZ() * tagToCam.getZ();

    double incidenceDeg = Math.toDegrees(Math.acos(MathUtil.clamp(dot / tagToCam.getNorm(), -1, 1)));

    if (incidenceDeg > settings.maxTagIncidenceAngleDegrees)
    {
      return Optional.empty();
    }

    double fx = (settings.resolutionWidth / 2.0) / Math.tan(Math.toRadians(halfHFovDeg));

    double apparentWidthPx = fx * settings.tagSizeMeters * Math.cos(Math.toRadians(incidenceDeg)) / x;

    double areaPx = apparentWidthPx * apparentWidthPx;

    double ta = MathUtil.clamp(areaPx / (settings.resolutionWidth * settings.resolutionHeight) * 100.0, 0, 100);

    double noisyYaw = yawDeg + noise.nextGaussian() * settings.angleNoiseStdDevDegrees;

    double noisyPitch = pitchDeg + noise.nextGaussian() * settings.angleNoiseStdDevDegrees;

    double distToRobot = robotPose.getTranslation().getDistance(tagPose.getTranslation());

    double ambiguity = MathUtil.clamp(noise.nextDouble() * 0.05 + (incidenceDeg / 90.0) * 0.15, 0, 1);

    double pixelsPerDegree = (settings.resolutionWidth / 2.0) / halfHFovDeg;

    return Optional.of(new TagObservation(tag.ID, tagPose, ta, noisyYaw, noisyPitch, distance, distToRobot, ambiguity, apparentWidthPx, pixelsPerDegree));
  }


 /**
   * Draw the simulated robot-to-tag raycasts onto the configured Field2d.
   *
   * <p>
   * A single FieldObject2d is used. The poses are ordered as:
   *
   * <pre>
   * robot -> tag1 -> robot -> tag2 -> robot -> tag3 -> ...
   * </pre>
   *
   * <p>
   * This produces separate-looking rays because the robot position is inserted between each tag.
   *
   * @param robotPose Ground-truth robot pose.
   * @param visible   Currently visible AprilTags.
   */
  private void drawRaycasts(Pose3d robotPose, List<TagObservation> visible)
  {
    if (raycasts == null)
    {
      return;
    }

    Pose2d robot = robotPose.toPose2d();

    if (visible.isEmpty())
    {
      raycasts.setPoses();
      return;
    }

    List<Pose2d> poses = new ArrayList<>(visible.size() * 2);

    for (TagObservation observation : visible)
    {
      poses.add(robot);
      poses.add(observation.pose.toPose2d());
    }

    raycasts.setPoses(poses.toArray(new Pose2d[0]));
  }

    /**
   * Set the {@link Field2d} used to visualize simulated AprilTag raycasts.
   *
   * <p>
   * The simulator does not set the built-in robot pose on the {@link Field2d}. Instead, it creates a single {@link FieldObject2d} named {@code "Limelight Raycasts"} containing the path:
   *
   * <pre>
   * robot -> tag1 -> robot -> tag2 -> robot -> ...
   * </pre>
   *
   * <p>
   * If {@code null} is supplied, visualization is disabled and any existing raycast path is cleared.
   *
   * @param field2d {@link Field2d} to draw the raycasts on.
   * @return {@link LimelightSim} for chaining.
   */
  public VisionIOSim withField2d(Field2d field2d)
  {
    if (field2d == null)
    {
      raycasts = null;
    } else
    {
      raycasts = field2d.getObject("Limelight Raycasts");
      raycasts.setPoses();
    }

    return this;
  }

}