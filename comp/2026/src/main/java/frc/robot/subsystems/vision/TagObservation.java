package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

/**
   * A single simulated tag detection.
   */
  public class TagObservation
  {

    final int    id;
    final Pose3d pose;
    final double ta;
    final double tx;
    final double ty;
    final double distToCamera;
    final double distToRobot;
    final double ambiguity;
    final double apparentWidthPixels;
    final double pixelsPerDegree;

    TagObservation(int id, Pose3d pose, double ta, double tx, double ty, double distToCamera, double distToRobot, double ambiguity, double apparentWidthPixels, double pixelsPerDegree)
    {
      this.id = id;
      this.pose = pose;
      this.ta = ta;
      this.tx = tx;
      this.ty = ty;
      this.distToCamera = distToCamera;
      this.distToRobot = distToRobot;
      this.ambiguity = ambiguity;
      this.apparentWidthPixels = apparentWidthPixels;
      this.pixelsPerDegree = pixelsPerDegree;
    }
  }
