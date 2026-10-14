package frc.vision.cal;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

/**
 * Where each tag really is, from detections at known robot poses using the configured mounts.
 * Mount errors and tag errors look alike, so each estimate carries the per-camera spread: if two
 * cameras put a tag in the same place, the tag (not a camera) is the likely problem.
 */
public final class TagEstimator {
  /** A detection already converted to a field position. */
  public record Sighting(int tagId, String camera, String spot, Translation3d fieldPos) {}

  public record Estimate(
      int tagId,
      Translation3d position, // median over all sightings
      int sightings,
      Set<String> cameras,
      Set<String> spots,
      double cameraSpreadM, // max distance between per-camera medians
      Translation3d layoutPosition, // null if the layout lacks this tag
      boolean confident) {
    public double offsetFromLayoutM() {
      return layoutPosition == null ? Double.NaN : position.getDistance(layoutPosition);
    }
  }

  public static double misplacedM = 0.05;

  public static List<Estimate> estimate(List<Sighting> all, AprilTagFieldLayout layout) {
    Map<Integer, List<Sighting>> byTag = new TreeMap<>();
    for (Sighting s : all) {
      byTag.computeIfAbsent(s.tagId(), k -> new ArrayList<>()).add(s);
    }
    List<Estimate> out = new ArrayList<>();
    for (var e : byTag.entrySet()) {
      List<Sighting> ss = e.getValue();
      Translation3d med = median(ss.stream().map(Sighting::fieldPos).toList());
      Map<String, List<Translation3d>> byCam = new TreeMap<>();
      Set<String> spots = new TreeSet<>();
      for (Sighting s : ss) {
        byCam.computeIfAbsent(s.camera(), k -> new ArrayList<>()).add(s.fieldPos());
        spots.add(s.spot());
      }
      List<Translation3d> camMeds = byCam.values().stream().map(TagEstimator::median).toList();
      double spread = 0;
      for (Translation3d a : camMeds) {
        for (Translation3d b : camMeds) {
          spread = Math.max(spread, a.getDistance(b));
        }
      }
      Translation3d layoutPos = layout.getTagPose(e.getKey()).map(Pose3d::getTranslation).orElse(null);
      boolean confident =
          ss.size() >= 10 && (byCam.size() >= 2 || spots.size() >= 2) && spread < misplacedM;
      out.add(
          new Estimate(
              e.getKey(), med, ss.size(), byCam.keySet(), spots, spread, layoutPos, confident));
    }
    return out;
  }

  /**
   * New layout: confident estimates replace tag positions (orientation kept from the base
   * layout, since detections here only give position); other tags stay as they were.
   */
  public static AprilTagFieldLayout updatedLayout(AprilTagFieldLayout base, List<Estimate> ests) {
    List<AprilTag> tags = new ArrayList<>();
    for (AprilTag t : base.getTags()) {
      Pose3d pose = t.pose;
      for (Estimate e : ests) {
        if (e.tagId() == t.ID && e.confident()) {
          pose = new Pose3d(e.position(), t.pose.getRotation());
        }
      }
      tags.add(new AprilTag(t.ID, pose));
    }
    return new AprilTagFieldLayout(tags, base.getFieldLength(), base.getFieldWidth());
  }

  static Translation3d median(List<Translation3d> pts) {
    double[] x = pts.stream().mapToDouble(Translation3d::getX).toArray();
    double[] y = pts.stream().mapToDouble(Translation3d::getY).toArray();
    double[] z = pts.stream().mapToDouble(Translation3d::getZ).toArray();
    return new Translation3d(MountCheck.median(x), MountCheck.median(y), MountCheck.median(z));
  }
}
