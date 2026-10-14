package frc.vision.cal;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * Scores exposure/gain settings per camera and spot, then picks ONE setting per camera that works
 * across all the chosen spots (Limelights can't switch settings by shooting location).
 */
public final class SweepScorer {
  /** Metrics for one camera, spot and setting. */
  public record Cell(
      String camera,
      String spot,
      String setting,
      double exposure,
      int frames,
      double detectRate, // frames with a tag / all frames
      double meanTags,
      double jitterM, // MT1 x/y spread (std dev) while parked
      double fps,
      double ambiguity, // mean single-tag ambiguity (NaN if only multi-tag)
      double score) {}

  /** The setting chosen for a camera and how it did at each spot. */
  public record Choice(
      String camera,
      String setting,
      double worstScore,
      double weightedMean,
      Map<String, Double> scoreBySpot,
      Map<String, String> bestSettingBySpot,
      List<String> excludedSpots) {}

  /** A spot counts for a camera only if the camera saw tags there at its current settings. */
  public static double minBaselineDetect = 0.3;

  /** How much worst-spot score we give up to get a lower exposure (less motion blur). */
  public static double EXPOSURE_TOLERANCE = 0.05;

  /** Builds cells from sweep frames (setting labels carry "exp=..,gain=.."). */
  public static List<Cell> cells(List<Frame> frames, Map<String, Double> exposureBySetting) {
    Map<String, List<Frame>> groups = new LinkedHashMap<>();
    for (Frame f : frames) {
      groups.computeIfAbsent(f.camera + "|" + f.spot + "|" + f.setting, k -> new ArrayList<>()).add(f);
    }
    List<Cell> raw = new ArrayList<>();
    for (var g : groups.entrySet()) {
      List<Frame> fs = g.getValue();
      Frame f0 = fs.get(0);
      int n = fs.size();
      long withTag = fs.stream().filter(Frame::hasTags).count();
      double meanTags = fs.stream().mapToInt(f -> f.tags.size()).average().orElse(0);
      double jitter = jitter(fs);
      double fps = fs.stream().mapToDouble(f -> f.fps).filter(Double::isFinite).average().orElse(Double.NaN);
      double amb = fs.stream().mapToDouble(Frame::singleTagAmbiguity).filter(Double::isFinite).average().orElse(Double.NaN);
      raw.add(
          new Cell(
              f0.camera,
              f0.spot,
              f0.setting,
              exposureBySetting.getOrDefault(f0.setting, Double.NaN),
              n,
              n == 0 ? 0 : (double) withTag / n,
              meanTags,
              jitter,
              fps,
              amb,
              0));
    }
    // Score relative to the best at the same camera + spot (lighting differs per spot)
    List<Cell> out = new ArrayList<>();
    for (Cell c : raw) {
      double maxTags = 0;
      double maxFps = 0;
      for (Cell o : raw) {
        if (o.camera().equals(c.camera()) && o.spot().equals(c.spot())) {
          maxTags = Math.max(maxTags, o.meanTags());
          if (Double.isFinite(o.fps())) {
            maxFps = Math.max(maxFps, o.fps());
          }
        }
      }
      out.add(withScore(c, score(c, maxTags, maxFps)));
    }
    return out;
  }

  /**
   * 0..1. Visibility (how often and how many tags) x steadiness (MT1 jitter, 2 cm halves it) x
   * frame rate (vs the best at this spot).
   */
  static double score(Cell c, double maxTags, double maxFps) {
    if (c.frames() == 0 || maxTags <= 0) {
      return 0;
    }
    double visibility = c.detectRate() * (c.meanTags() / maxTags);
    double steadiness = Double.isFinite(c.jitterM()) ? 1.0 / (1.0 + c.jitterM() / 0.02) : 0.5;
    double fpsFactor =
        (Double.isFinite(c.fps()) && maxFps > 0) ? Math.min(1.0, c.fps() / (0.9 * maxFps)) : 1.0;
    // Single-tag ambiguity (0 = certain, 1 = coin flip) costs up to half the score
    double ambFactor = Double.isFinite(c.ambiguity()) ? 1.0 - 0.5 * Math.min(1.0, Math.max(0.0, c.ambiguity())) : 1.0;
    return visibility * steadiness * fpsFactor * ambFactor;
  }

  /**
   * One setting per camera: highest worst-spot score, then highest weighted mean, then lowest
   * exposure (less motion blur). Spots where the camera barely saw tags at its baseline settings
   * are excluded for that camera (nothing to tune there).
   */
  public static List<Choice> reconcile(
      List<Cell> cells, Map<String, Double> spotWeights, Map<String, Double> baselineDetect) {
    Map<String, List<Cell>> byCam = new TreeMap<>();
    for (Cell c : cells) {
      byCam.computeIfAbsent(c.camera(), k -> new ArrayList<>()).add(c);
    }
    List<Choice> out = new ArrayList<>();
    for (var e : byCam.entrySet()) {
      String cam = e.getKey();
      List<Cell> cs = e.getValue();
      List<String> spots = cs.stream().map(Cell::spot).distinct().toList();
      List<String> used = new ArrayList<>();
      List<String> excluded = new ArrayList<>();
      for (String s : spots) {
        double b = baselineDetect.getOrDefault(cam + "|" + s, 1.0);
        (b >= minBaselineDetect ? used : excluded).add(s);
      }
      if (used.isEmpty()) {
        out.add(new Choice(cam, null, 0, 0, Map.of(), Map.of(), excluded));
        continue;
      }
      List<String> settings = cs.stream().map(Cell::setting).distinct().toList();
      record Cand(String set, double worst, double mean, double exp, Map<String, Double> bySpot) {}
      List<Cand> cands = new ArrayList<>();
      for (String set : settings) {
        double worst = Double.POSITIVE_INFINITY;
        double wsum = 0;
        double sum = 0;
        Map<String, Double> bySpot = new LinkedHashMap<>();
        double exp = Double.NaN;
        boolean complete = true;
        for (String s : used) {
          Cell c = find(cs, s, set);
          if (c == null) {
            complete = false;
            break;
          }
          exp = c.exposure();
          double w = spotWeights.getOrDefault(s, 1.0);
          worst = Math.min(worst, c.score());
          sum += w * c.score();
          wsum += w;
          bySpot.put(s, c.score());
        }
        if (!complete) {
          continue;
        }
        double mean = wsum > 0 ? sum / wsum : 0;
        cands.add(new Cand(set, worst, mean, Double.isFinite(exp) ? exp : 0, bySpot));
      }
      // Limelight guidance: use the smallest exposure that still tracks reliably (less motion
      // blur). Among settings within EXPOSURE_TOLERANCE of the best worst-spot score, take the
      // lowest exposure; break ties by weighted mean.
      double topWorst = cands.stream().mapToDouble(Cand::worst).max().orElse(-1);
      Cand pick =
          cands.stream()
              .filter(c -> c.worst() >= topWorst * (1 - EXPOSURE_TOLERANCE) - 1e-9)
              .min(
                  Comparator.comparingDouble(Cand::exp)
                      .thenComparing(Comparator.comparingDouble(Cand::mean).reversed()))
              .orElse(null);
      String best = pick == null ? null : pick.set();
      double bestWorst = pick == null ? 0 : pick.worst();
      double bestMean = pick == null ? 0 : pick.mean();
      Map<String, Double> bestBySpot = pick == null ? null : pick.bySpot();
      Map<String, String> bestPerSpot = new LinkedHashMap<>();
      for (String s : used) {
        cs.stream()
            .filter(c -> c.spot().equals(s))
            .max(Comparator.comparingDouble(Cell::score))
            .ifPresent(c -> bestPerSpot.put(s, c.setting()));
      }
      out.add(
          new Choice(
              cam,
              best,
              bestWorst,
              bestMean,
              bestBySpot == null ? Map.of() : bestBySpot,
              bestPerSpot,
              excluded));
    }
    return out;
  }

  private static Cell find(List<Cell> cs, String spot, String setting) {
    for (Cell c : cs) {
      if (c.spot().equals(spot) && c.setting().equals(setting)) {
        return c;
      }
    }
    return null;
  }

  private static Cell withScore(Cell c, double s) {
    return new Cell(
        c.camera(), c.spot(), c.setting(), c.exposure(), c.frames(), c.detectRate(), c.meanTags(),
        c.jitterM(), c.fps(), c.ambiguity(), s);
  }

  /** Std dev of MT1 x/y (meters) across frames that had a pose. */
  static double jitter(List<Frame> fs) {
    List<double[]> p = fs.stream().map(f -> f.mt1).filter(m -> m != null).toList();
    if (p.size() < 3) {
      return Double.NaN;
    }
    double mx = p.stream().mapToDouble(a -> a[0]).average().orElse(0);
    double my = p.stream().mapToDouble(a -> a[1]).average().orElse(0);
    double s = 0;
    for (double[] a : p) {
      s += (a[0] - mx) * (a[0] - mx) + (a[1] - my) * (a[1] - my);
    }
    return Math.sqrt(s / p.size());
  }
}
