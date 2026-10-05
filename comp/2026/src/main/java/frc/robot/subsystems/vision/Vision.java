// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.vision.VisionConstants.*;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;


public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  // Timestamp of the last accepted observation per camera, to drop stale/repeated frames
  private final double[] lastAcceptedTimestamps;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    this.lastAcceptedTimestamps = new double[io.length];
    Arrays.fill(lastAcceptedTimestamps, Double.NEGATIVE_INFINITY);
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    for (limelightConstants camera : cameraConstants) {
      LimelightHelpers.setCameraPose_RobotSpace(camera.name, camera.forward, camera.side, camera.up, camera.roll, camera.pitch, camera.yaw);
    }
    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {

   
    

    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        //if (VisionConstants.IgnoreTags.contains(tagId)){
          //continue;
        //}
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      List<String> rejectReasons = new LinkedList<>();
      List<Double> linearStdDevs = new LinkedList<>();
      boolean spinningTooFast =
          Math.abs(Units.radiansToDegrees(RobotState.getInstance().yawRateRadPerSec))
              > maxYawRateDegPerSec;

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose (first matching reason is logged)
        String rejectReason = null;
        if (observation.tagCount() == 0) {
          rejectReason = "NoTags";
        } else if (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) {
          rejectReason = "Ambiguity";
        } else if (Math.abs(observation.pose().getZ()) > maxZError) {
          rejectReason = "ZError";
        } else if (observation.pose().getX() <= 0.0
            || observation.pose().getX() > aprilTagLayout.getFieldLength()
            || observation.pose().getY() <= 0.0
            || observation.pose().getY() > aprilTagLayout.getFieldWidth()) {
          rejectReason = "OffField";
        } else if (spinningTooFast) {
          rejectReason = "Spinning";
        } else if (observation.timestamp() <= lastAcceptedTimestamps[cameraIndex]) {
          rejectReason = "StaleFrame";
        }
        boolean rejectPose = rejectReason != null;

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
          rejectReasons.add(rejectReason);
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }
        lastAcceptedTimestamps[cameraIndex] = observation.timestamp();

        // Calculate standard deviations
        double linearStdDev = linearStdDevMegatag2Base;
        double angularStdDev = angularStdDevMegatag2Base;
        if (useDistanceScaledStdDevs) {
          // Trust drops with distance squared and rises with more tags
          double stdDevFactor =
              Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
          linearStdDev = linearStdDevBaseline * stdDevFactor;
          angularStdDev = angularStdDevBaseline * stdDevFactor;
          if (observation.type() == PoseObservationType.MEGATAG_2) {
            linearStdDev *= linearStdDevMegatag2Factor;
            angularStdDev *= angularStdDevMegatag2Factor;
          }
          if (cameraIndex < cameraStdDevFactors.length) {
            linearStdDev *= cameraStdDevFactors[cameraIndex];
            angularStdDev *= cameraStdDevFactors[cameraIndex];
          }
        }
        linearStdDevs.add(linearStdDev);

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera metadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RejectReasons",
          rejectReasons.toArray(new String[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/AcceptedLinearStdDevs",
          linearStdDevs.stream().mapToDouble(Double::doubleValue).toArray());
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/ExternalAngle",
          inputs[cameraIndex].externalAngle);


      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public Rotation2d getShooterTx(){
    return inputs[0].latestTargetObservation.tx();
  }


}