package frc.vision.cal;

import com.fasterxml.jackson.databind.JsonNode;
import java.util.ArrayList;
import java.util.List;

/**
 * One Limelight result, reduced to what calibration needs. Stored as JSON lines in a run folder
 * so reports can be re-run later without the robot.
 */
public final class Frame {
  /** One AprilTag detection. */
  public static final class Tag {
    public int id;
    public double txnc; // deg, right-positive, from principal point
    public double tync; // deg, up-positive
    public double distM; // camera to tag center
    public double area;
    public double ambiguity = Double.NaN;

    public Tag() {}

    public Tag(int id, double txnc, double tync, double distM, double area) {
      this.id = id;
      this.txnc = txnc;
      this.tync = tync;
      this.distM = distM;
      this.area = area;
    }
  }

  /**
   * Builds a frame from Limelight NetworkTables arrays (documented in the Complete NetworkTables
   * API): botpose_wpiblue / botpose_orb_wpiblue = [x, y, z, roll, pitch, yaw, latency, tagCount,
   * tagSpan, avgDist, avgArea]; rawfiducials = groups of [id, txnc, tync, ta, distToCamera,
   * distToRobot, ambiguity]; hw = [cpuTemp, cpu, ram, fps].
   */
  public static Frame fromNt(
      String camera, double ntTimeUs, double[] mt1, double[] mt2, double[] raw, double[] hw) {
    Frame f = new Frame();
    f.camera = camera;
    f.wallTime = System.currentTimeMillis() / 1000.0;
    f.llTimestampMs = ntTimeUs / 1000.0;
    if (raw != null) {
      for (int i = 0; i + 6 < raw.length; i += 7) {
        Tag t = new Tag((int) raw[i], raw[i + 1], raw[i + 2], raw[i + 4], raw[i + 3]);
        t.ambiguity = raw[i + 6];
        if (t.distM > 0) {
          f.tags.add(t);
        }
      }
    }
    f.mt1 = f.tags.isEmpty() ? null : pose6(mt1);
    f.mt2 = f.tags.isEmpty() ? null : pose6(mt2);
    if (mt1 != null && mt1.length >= 10) {
      f.tagCount = (int) mt1[7];
      f.avgTagDistM = mt1[9];
    }
    if (f.tags.isEmpty()) {
      f.tagCount = 0;
    } else if (f.tagCount == 0) {
      f.tagCount = f.tags.size();
    }
    if (hw != null && hw.length >= 4) {
      f.fps = hw[3];
    }
    return f;
  }

  /** Ambiguity of the frame's single tag (NaN for multi-tag or unknown). */
  public double singleTagAmbiguity() {
    return tags.size() == 1 ? tags.get(0).ambiguity : Double.NaN;
  }

  public String camera;
  public String spot; // spot id
  public String phase; // "baseline" or "sweep"
  public String setting; // sweep setting label, "current" for baseline
  public double wallTime; // laptop clock, seconds
  public double llTimestampMs; // Limelight "ts", used to drop repeated frames
  public double fps = Double.NaN;
  public double[] mt1; // x, y, z, roll, pitch, yaw (deg), WPILib blue origin; null if none
  public double[] mt2;
  public int tagCount;
  public double avgTagDistM;
  public List<Tag> tags = new ArrayList<>();

  public boolean hasTags() {
    return !tags.isEmpty();
  }

  /**
   * Parses a /results response. Handles both the flat format and the older {"Results": {...}}
   * wrapper. Distance comes from the length of t6t_cs (tag pose in camera space), which doesn't
   * depend on Limelight's rotation conventions.
   */
  public static Frame parse(JsonNode root, String camera) {
    JsonNode r = root.has("Results") ? root.get("Results") : root;
    Frame f = new Frame();
    f.camera = camera;
    f.wallTime = System.currentTimeMillis() / 1000.0;
    f.llTimestampMs = r.path("ts").asDouble(Double.NaN);
    boolean valid = r.path("v").asInt(1) == 1;
    f.mt1 = pose6(r.path("botpose_wpiblue"));
    f.mt2 = pose6(r.path("botpose_orb_wpiblue"));
    f.tagCount = r.path("botpose_tagcount").asInt(0);
    f.avgTagDistM = r.path("botpose_avgdist").asDouble(0);
    for (JsonNode fid : r.path("Fiducial")) {
      JsonNode cs = fid.path("t6t_cs");
      double dist =
          cs.size() >= 3
              ? Math.sqrt(
                  cs.get(0).asDouble() * cs.get(0).asDouble()
                      + cs.get(1).asDouble() * cs.get(1).asDouble()
                      + cs.get(2).asDouble() * cs.get(2).asDouble())
              : Double.NaN;
      double txnc = fid.has("tx_nocross") ? fid.get("tx_nocross").asDouble() : fid.path("tx").asDouble();
      double tync = fid.has("ty_nocross") ? fid.get("ty_nocross").asDouble() : fid.path("ty").asDouble();
      if (Double.isFinite(dist) && dist > 0) {
        f.tags.add(new Tag(fid.path("fID").asInt(-1), txnc, tync, dist, fid.path("ta").asDouble()));
      }
    }
    if (!valid || f.tags.isEmpty() || f.tagCount == 0) {
      f.mt1 = null;
      f.mt2 = null;
      if (f.tags.isEmpty()) {
        f.tagCount = 0;
      }
    }
    if (f.tagCount == 0) {
      f.tagCount = f.tags.size();
    }
    return f;
  }

  private static double[] pose6(JsonNode arr) {
    if (arr == null || arr.size() < 6) {
      return null;
    }
    double[] p = new double[6];
    for (int i = 0; i < 6; i++) {
      p[i] = arr.get(i).asDouble();
    }
    return pose6(p);
  }

  private static double[] pose6(double[] arr) {
    if (arr == null || arr.length < 6) {
      return null;
    }
    double[] p = java.util.Arrays.copyOf(arr, 6);
    // Limelight reports all zeros when it has no pose
    if (p[0] == 0 && p[1] == 0 && p[5] == 0) {
      return null;
    }
    return p;
  }
}
