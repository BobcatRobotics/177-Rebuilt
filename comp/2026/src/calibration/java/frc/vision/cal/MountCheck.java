package frc.vision.cal;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Sanity-checks a camera's configured mount (from CAD / team measurements, which stay the source
 * of truth). It never computes new offsets. It answers: does this camera's view agree with where
 * the robot really is, and if not, does a common setup mistake explain it?
 */
public final class MountCheck {
  /** One detection taken while the robot sat at a known pose. */
  public record Obs(Pose3d robot, Translation3d tagInCam, Translation3d tagField, int tagId, String spot) {}

  public enum Verdict {
    OK,
    SUSPICIOUS,
    WRONG,
    NO_DATA
  }

  /** A configured mount with one classic mistake undone, and how well it fits. */
  public record Hypothesis(String description, double[] llMount, double rmsM) {}

  public record Result(
      Verdict verdict,
      double rmsM, // tag position error using the configured mount
      double medianM,
      int observations,
      List<Hypothesis> betterFits, // only hypotheses that fit far better than configured
      List<String> notes) {}

  /** Thresholds; defaults can be overridden from the command line. */
  public static double suspiciousM = 0.08;
  public static double wrongM = 0.50;

  /**
   * @param ll configured Limelight mount {forward, side, up, roll, pitch, yaw}
   */
  public static Result check(double[] ll, List<Obs> obs) {
    List<String> notes = new ArrayList<>(staticNotes(ll));
    if (obs.isEmpty()) {
      return new Result(Verdict.NO_DATA, Double.NaN, Double.NaN, 0, List.of(), notes);
    }
    double[] errs = errors(ll, obs);
    double rms = rms(errs);
    double med = median(errs);

    List<Hypothesis> better = new ArrayList<>();
    for (Hypothesis h : hypotheses(ll)) {
      double r = rms(errors(h.llMount(), obs));
      // "Far better": at least 3x smaller error and actually good
      if (r < rms / 3 && r < Math.max(suspiciousM, rms / 2)) {
        better.add(new Hypothesis(h.description(), h.llMount(), r));
      }
    }
    better.sort((a, b) -> Double.compare(a.rmsM(), b.rmsM()));

    Verdict v;
    if (med > wrongM || !better.isEmpty() && rms > suspiciousM) {
      v = Verdict.WRONG;
    } else if (med > suspiciousM) {
      v = Verdict.SUSPICIOUS;
    } else {
      v = Verdict.OK;
    }
    return new Result(v, rms, med, obs.size(), better, notes);
  }

  /** Common setup mistakes to test, each applied to the configured values. */
  static List<Hypothesis> hypotheses(double[] ll) {
    List<Hypothesis> hs = new ArrayList<>();
    hs.add(variant("yaw is 180° off", ll, 5, wrap(ll[5] + 180)));
    if (Math.abs(ll[5]) > 1) {
      hs.add(variant("yaw sign is flipped", ll, 5, -ll[5]));
    }
    if (Math.abs(ll[1]) > 0.01) {
      hs.add(variant("side sign is flipped (left/right)", ll, 1, -ll[1]));
    }
    if (Math.abs(ll[4]) > 1) {
      hs.add(variant("pitch sign is flipped (up/down)", ll, 4, -ll[4]));
    }
    if (Math.abs(ll[0]) > 0.01 || Math.abs(ll[1]) > 0.01) {
      double[] s = ll.clone();
      s[0] = ll[1];
      s[1] = ll[0];
      hs.add(new Hypothesis("forward and side are swapped", s, Double.NaN));
    }
    double[] inch = ll.clone();
    boolean anyBig = false;
    for (int i = 0; i < 3; i++) {
      if (Math.abs(ll[i]) > 1.0) {
        inch[i] = ll[i] * 0.0254;
        anyBig = true;
      }
    }
    if (anyBig) {
      hs.add(new Hypothesis("some offsets were entered in inches, not meters", inch, Double.NaN));
    }
    return hs;
  }

  /** Checks that need no field data. */
  static List<String> staticNotes(double[] ll) {
    List<String> n = new ArrayList<>();
    for (int i = 0; i < 3; i++) {
      if (Math.abs(ll[i]) > 1.0) {
        n.add(
            String.format(
                "%s = %.3f m is bigger than the robot; was it entered in inches?",
                new String[] {"forward", "side", "up"}[i], ll[i]));
      }
    }
    if (ll[2] < 0 || ll[2] > 1.5) {
      n.add(String.format("height (up) = %.3f m looks implausible", ll[2]));
    }
    if (Math.abs(ll[4]) > 60) {
      n.add(String.format("pitch = %.1f° is very steep; check its sign/units", ll[4]));
    }
    return n;
  }

  /**
   * Camera pose saved inside a Limelight pipeline (rsf, rss, rsu, rsroll, rspitch, rsyaw — same
   * order and conventions as VisionConstants), or null if the pipeline has none.
   */
  public static double[] pipelineMount(com.fasterxml.jackson.databind.JsonNode pipe) {
    String[] keys = {"rsf", "rss", "rsu", "rsroll", "rspitch", "rsyaw"};
    double[] m = new double[6];
    for (int i = 0; i < 6; i++) {
      if (pipe == null || !pipe.has(keys[i])) {
        return null;
      }
      m[i] = pipe.get(keys[i]).asDouble();
    }
    return m;
  }

  /**
   * Differences between the robot code's mount and the one saved on the camera. Robot code pushes
   * its own pose at startup (setCameraPose_RobotSpace), so the saved one only matters when robot
   * code isn't running — but a mismatch means someone measured twice and got different answers.
   */
  public static List<String> compareToPipeline(double[] code, double[] saved) {
    List<String> n = new ArrayList<>();
    if (saved == null) {
      return n;
    }
    String[] names = {"forward", "side", "up", "roll", "pitch", "yaw"};
    for (int i = 0; i < 6; i++) {
      double d = i < 3 ? Math.abs(code[i] - saved[i]) : Math.abs(wrap(code[i] - saved[i]));
      if (i < 3 ? d > 0.02 : d > 2.0) {
        n.add(String.format(
            "%s: code %s vs saved on camera %s%s",
            names[i],
            i < 3 ? String.format("%.3f", code[i]) : String.format("%.1f", code[i]),
            i < 3 ? String.format("%.3f", saved[i]) : String.format("%.1f", saved[i]),
            i < 3 && Math.signum(code[i]) != Math.signum(saved[i]) && Math.abs(code[i]) > 0.02
                    && Math.abs(saved[i]) > 0.02
                ? " (opposite sign!)"
                : ""));
      }
    }
    return n;
  }

  /** Flags cameras outside the bumper outline (robot length/width in meters). */
  public static String outsideBumpers(double[] ll, double lengthM, double widthM) {
    if (Math.abs(ll[0]) > lengthM / 2 + 0.02 || Math.abs(ll[1]) > widthM / 2 + 0.02) {
      return String.format(
          "camera at forward %.3f, side %.3f is outside the %.2f x %.2f m bumpers",
          ll[0], ll[1], lengthM, widthM);
    }
    return null;
  }

  /**
   * Describes the average pose error of Limelight MegaTag1 poses vs truth in robot terms, e.g.
   * "reads 12 cm too far forward, 3 cm too far left, heading +9°".
   */
  public static String describeBias(Pose2d truth, List<double[]> mt1Poses) {
    if (mt1Poses.isEmpty()) {
      return "no pose";
    }
    double dx = 0;
    double dy = 0;
    double dh = 0;
    for (double[] p : mt1Poses) {
      dx += p[0] - truth.getX();
      dy += p[1] - truth.getY();
      dh += wrap(p[5] - truth.getRotation().getDegrees());
    }
    int n = mt1Poses.size();
    Translation2d robotFrame =
        new Translation2d(dx / n, dy / n).rotateBy(truth.getRotation().unaryMinus());
    return String.format(
        "%s, %s, heading %+.1f°",
        dir(robotFrame.getX(), "forward", "back"),
        dir(robotFrame.getY(), "left", "right"),
        dh / n);
  }

  private static String dir(double m, String pos, String neg) {
    if (Math.abs(m) < 0.02) {
      return "~0 cm " + pos + "/" + neg;
    }
    return String.format("%.0f cm too far %s", Math.abs(m) * 100, m > 0 ? pos : neg);
  }

  static double[] errors(double[] ll, List<Obs> obs) {
    Transform3d m = CamGeom.mountFromLimelight(ll[0], ll[1], ll[2], ll[3], ll[4], ll[5]);
    double[] e = new double[obs.size()];
    for (int i = 0; i < obs.size(); i++) {
      Obs o = obs.get(i);
      e[i] = CamGeom.tagInField(o.robot(), m, o.tagInCam()).getDistance(o.tagField());
    }
    return e;
  }

  private static Hypothesis variant(String desc, double[] ll, int idx, double value) {
    double[] v = ll.clone();
    v[idx] = value;
    return new Hypothesis(desc, v, Double.NaN);
  }

  static double wrap(double deg) {
    double d = ((deg + 180) % 360 + 360) % 360 - 180;
    return d;
  }

  static double rms(double[] v) {
    double s = 0;
    for (double x : v) {
      s += x * x;
    }
    return v.length == 0 ? Double.NaN : Math.sqrt(s / v.length);
  }

  static double median(double[] v) {
    if (v.length == 0) {
      return Double.NaN;
    }
    double[] s = v.clone();
    Arrays.sort(s);
    return s.length % 2 == 1 ? s[s.length / 2] : (s[s.length / 2 - 1] + s[s.length / 2]) / 2;
  }
}
