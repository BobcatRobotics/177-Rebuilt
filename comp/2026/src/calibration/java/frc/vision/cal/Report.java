package frc.vision.cal;

import com.fasterxml.jackson.databind.JsonNode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/** Builds report.md for a run. Every number in it can be recomputed from the run folder. */
public final class Report {
  public static double crossCameraM = 0.10;
  public static double crossCameraDeg = 5.0;
  public static double trustedErrorM = 0.10;
  static final double[] BINS = {0, 1.5, 2.5, 3.5, 4.5, 6.0, 99};

  /** Everything the report needs, loaded from a run. */
  public record Inputs(
      FieldConfig field,
      List<Cameras.Camera> cameras,
      Map<String, Pose2d> truth, // spot id -> pose (complete spots only)
      Map<String, Double> weights,
      Map<String, Pose2d> odometry,
      List<Frame> frames,
      Map<String, Double> exposureByCameraSetting, // "cam|label" -> exposure
      Map<String, JsonNode> pipelines, // camera -> pipeline JSON backed up before the run
      JsonNode confirmations) {} // y/n answers to "is the camera mounted/pointing like this?"

  public static String build(Inputs in) {
    StringBuilder md = new StringBuilder();
    md.append("# Limelight calibration report\n\n");
    md.append("Field: **").append(in.field().name).append("** (layout `")
        .append(in.field().layoutSource).append("`). Spots recorded: ")
        .append(String.join(", ", in.truth().keySet())).append(".\n\n");
    md.append("Mount offsets come from CAD / team measurements in `VisionConstants`. This report ")
        .append("only checks them; it never computes new offsets.\n\n");

    List<Frame> baseline = in.frames().stream().filter(f -> f.phase.equals("baseline")).toList();

    staticChecks(md, in);
    pipelineChecks(md, in);
    mountChecks(md, in, baseline);
    crossCamera(md, in, baseline);
    trustVsDistance(md, in, baseline);
    tags(md, in, baseline);
    sweep(md, in);
    odometry(md, in);
    return md.toString();
  }

  private static void staticChecks(StringBuilder md, Inputs in) {
    md.append("## 1. Configured mounts (no field data needed)\n\n");
    md.append("| Camera | forward | side (right +) | up | roll | pitch (up +) | yaw (left +) | Notes |\n");
    md.append("|---|---|---|---|---|---|---|---|\n");
    for (Cameras.Camera c : in.cameras()) {
      double[] m = c.llMount();
      List<String> notes = new ArrayList<>(MountCheck.staticNotes(m));
      String outside = MountCheck.outsideBumpers(m, in.field().robotLengthM, in.field().robotWidthM);
      if (outside != null) {
        notes.add(outside);
      }
      md.append(String.format(
          "| %s | %.3f | %.3f | %.3f | %.1f | %.1f | %.1f | %s |%n",
          c.name(), m[0], m[1], m[2], m[3], m[4], m[5], notes.isEmpty() ? "OK" : String.join("; ", notes)));
    }
    md.append('\n');
  }

  /**
   * Pipeline settings vs Limelight's AprilTag guidance: black level 0, moderate gain (quick start
   * uses ~15; higher gain hurts 3D stability), highest resolution for 3D.
   */
  private static void pipelineChecks(StringBuilder md, Inputs in) {
    if (in.pipelines().isEmpty()) {
      return;
    }
    md.append("## 1b. Pipeline settings vs Limelight's AprilTag guidance\n\n");
    for (var e : in.pipelines().entrySet()) {
      JsonNode p = e.getValue();
      List<String> notes = new ArrayList<>();
      List<String> shown = new ArrayList<>();
      for (var it = p.fieldNames(); it.hasNext(); ) {
        String k = it.next();
        String l = k.toLowerCase();
        JsonNode v = p.get(k);
        if (!v.isValueNode()) {
          continue;
        }
        if (l.contains("black")) {
          shown.add(k + "=" + v.asText());
          if (v.isNumber() && v.asDouble() != 0) {
            notes.add("black level should be 0 (Limelight recommendation)");
          }
        } else if (l.contains("gain")) {
          shown.add(k + "=" + v.asText());
        } else if (l.contains("expos") || l.contains("res") || l.contains("downscale")) {
          shown.add(k + "=" + v.asText());
        }
      }
      Cameras.Camera cam = in.cameras().stream().filter(c -> c.name().equals(e.getKey())).findFirst().orElse(null);
      if (cam != null) {
        for (String d : MountCheck.compareToPipeline(cam.llMount(), MountCheck.pipelineMount(p))) {
          notes.add("pose saved on the camera differs from VisionConstants (the source of truth; robot code pushes it at startup) - " + d);
        }
      }
      md.append("- **").append(e.getKey()).append("**: ").append(String.join(", ", shown));
      if (!notes.isEmpty()) {
        md.append(" — ").append(String.join("; ", notes));
      }
      md.append('\n');
    }
    md.append("\nLimelight recommends black level 0, moderate gain (higher gain can reduce 3D "
        + "stability), the highest resolution for 3D tracking, and the smallest exposure that still tracks "
        + "reliably (less motion blur). Robot code pushes its own mount at startup, so a mismatch with the "
        + "pose saved on the camera only matters without robot code, but it means the two measurements disagree.\n\n");
  }

  static List<MountCheck.Obs> observations(Inputs in, List<Frame> baseline, String camera) {
    List<MountCheck.Obs> obs = new ArrayList<>();
    for (Frame f : baseline) {
      Pose2d t = in.truth().get(f.spot);
      if (t == null || !f.camera.equals(camera)) {
        continue;
      }
      for (Frame.Tag tag : f.tags) {
        var pos = in.field().tagPosition(tag.id);
        if (pos.isEmpty()) {
          continue;
        }
        obs.add(new MountCheck.Obs(
            new Pose3d(t), CamGeom.tagInCamera(tag.txnc, tag.tync, tag.distM), pos.get(), tag.id, f.spot));
      }
    }
    return obs;
  }

  private static void mountChecks(StringBuilder md, Inputs in, List<Frame> baseline) {
    md.append("## 2. Do the cameras agree with where the robot really is?\n\n");
    md.append(String.format(
        "Verdicts: OK under %.0f cm median tag error, SUSPICIOUS above, WRONG above %.0f cm or when a "
            + "common setup mistake explains the data far better.%n%n",
        MountCheck.suspiciousM * 100, MountCheck.wrongM * 100));
    for (Cameras.Camera c : in.cameras()) {
      List<MountCheck.Obs> obs = observations(in, baseline, c.name());
      MountCheck.Result r = MountCheck.check(c.llMount(), obs);
      List<String> said = answers(in, c.name(), "n");
      md.append("### ").append(c.name()).append(" — **").append(r.verdict()).append("**")
          .append(said.isEmpty() ? "" : " — **you said its mount/direction does not match the robot**").append("\n\n");
      for (String q : said) {
        md.append("- You answered **no** to: ").append(q).append('\n');
      }
      if (r.verdict() == MountCheck.Verdict.NO_DATA) {
        md.append("No tags with known positions were seen from any recorded spot.\n\n");
        continue;
      }
      md.append(String.format(
          "- Tag position error with the configured mount: median %.1f cm, RMS %.1f cm (%d detections)%n",
          r.medianM() * 100, r.rmsM() * 100, r.observations()));
      for (MountCheck.Hypothesis h : r.betterFits()) {
        md.append(String.format(
            "- **Likely setup mistake: %s.** With that undone the error drops to %.1f cm "
                + "(would be forward %.3f, side %.3f, up %.3f, roll %.1f, pitch %.1f, yaw %.1f). "
                + "Check against CAD before changing anything.%n",
            h.description(), h.rmsM() * 100, h.llMount()[0], h.llMount()[1], h.llMount()[2],
            h.llMount()[3], h.llMount()[4], h.llMount()[5]));
      }
      for (String spot : in.truth().keySet()) {
        List<double[]> mt1 = poses(baseline, c.name(), spot, true);
        List<double[]> mt2 = poses(baseline, c.name(), spot, false);
        if (mt1.isEmpty() && mt2.isEmpty()) {
          md.append("- ").append(spot).append(": no pose (no tags seen)\n");
          continue;
        }
        md.append("- ").append(spot).append(": MegaTag1 ")
            .append(MountCheck.describeBias(in.truth().get(spot), mt1));
        if (!mt2.isEmpty()) {
          String b = MountCheck.describeBias(in.truth().get(spot), mt2);
          md.append("; MegaTag2 ").append(b.substring(0, b.lastIndexOf(", heading")));
        }
        md.append(String.format(" (%d frames)%n", Math.max(mt1.size(), mt2.size())));
      }
      md.append('\n');
    }
  }

  private static void crossCamera(StringBuilder md, Inputs in, List<Frame> baseline) {
    md.append("## 3. Do the cameras agree with each other?\n\n");
    md.append(String.format(
        "Per spot, each camera's median MegaTag1 pose vs the median of all cameras. Flagged when "
            + "more than %.0f cm or %.0f° from the others.%n%n", crossCameraM * 100, crossCameraDeg));
    boolean any = false;
    for (String spot : in.truth().keySet()) {
      Map<String, double[]> med = new LinkedHashMap<>();
      for (Cameras.Camera c : in.cameras()) {
        List<double[]> p = poses(baseline, c.name(), spot, true);
        if (!p.isEmpty()) {
          med.put(c.name(), medianPose(p));
        }
      }
      if (med.size() < 2) {
        continue;
      }
      any = true;
      double[] all = medianPose(new ArrayList<>(med.values()));
      md.append("- ").append(spot).append(": ");
      List<String> parts = new ArrayList<>();
      for (var e : med.entrySet()) {
        double d = Math.hypot(e.getValue()[0] - all[0], e.getValue()[1] - all[1]);
        double h = MountCheck.wrap(e.getValue()[5] - all[5]);
        boolean flag = d > crossCameraM || Math.abs(h) > crossCameraDeg;
        parts.add(String.format("%s %s%.0f cm / %+.1f°%s", e.getKey(), flag ? "**" : "", d * 100, h, flag ? "**" : ""));
      }
      md.append(String.join(", ", parts)).append('\n');
    }
    if (!any) {
      md.append("No spot had two or more cameras seeing tags.\n");
    }
    md.append('\n');
  }

  private static void trustVsDistance(StringBuilder md, Inputs in, List<Frame> baseline) {
    md.append("## 4. How far out can each camera be trusted?\n\n");
    md.append("MegaTag1 position error vs truth, binned by average tag distance (median cm / frames).\n\n");
    StringBuilder header = new StringBuilder("| Camera |");
    StringBuilder sep = new StringBuilder("|---|");
    for (int b = 0; b + 1 < BINS.length; b++) {
      header.append(String.format(" %s–%s m |", fmt(BINS[b]), BINS[b + 1] > 50 ? "+" : fmt(BINS[b + 1])));
      sep.append("---|");
    }
    md.append(header).append(" Trusted to |\n").append(sep).append("---|\n");
    Map<String, Double> camMedian = new LinkedHashMap<>();
    for (Cameras.Camera c : in.cameras()) {
      md.append("| ").append(c.name()).append(" |");
      List<Double> allErr = new ArrayList<>();
      double trusted = 0;
      boolean stillGood = true;
      for (int b = 0; b + 1 < BINS.length; b++) {
        List<Double> errs = new ArrayList<>();
        for (Frame f : baseline) {
          Pose2d t = in.truth().get(f.spot);
          if (t == null || !f.camera.equals(c.name()) || f.mt1 == null) {
            continue;
          }
          if (f.avgTagDistM >= BINS[b] && f.avgTagDistM < BINS[b + 1]) {
            errs.add(Math.hypot(f.mt1[0] - t.getX(), f.mt1[1] - t.getY()));
          }
        }
        allErr.addAll(errs);
        if (errs.isEmpty()) {
          md.append(" — |");
          continue;
        }
        double m = MountCheck.median(errs.stream().mapToDouble(Double::doubleValue).toArray());
        md.append(String.format(" %.0f / %d |", m * 100, errs.size()));
        if (stillGood && m <= trustedErrorM) {
          trusted = Math.min(BINS[b + 1], 6.0);
        } else {
          stillGood = false;
        }
      }
      md.append(trusted > 0 ? String.format(" %.1f m |%n", trusted) : " — |\n");
      if (!allErr.isEmpty()) {
        camMedian.put(c.name(), MountCheck.median(allErr.stream().mapToDouble(Double::doubleValue).toArray()));
      }
    }
    if (camMedian.size() >= 2) {
      double ref = MountCheck.median(camMedian.values().stream().mapToDouble(Double::doubleValue).toArray());
      md.append("\nSuggested relative trust (`VisionConstants.cameraStdDevFactors`, 1.0 = typical "
          + "camera, higher = trust less), from each camera's median error vs the others:\n\n");
      for (var e : camMedian.entrySet()) {
        double f = Math.max(0.5, Math.round(e.getValue() / Math.max(ref, 1e-3) * 10) / 10.0);
        md.append(String.format("- %s: %.1f (median error %.0f cm)%n", e.getKey(), f, e.getValue() * 100));
      }
      md.append("\nOnly a suggestion; it assumes the spots' truth poses are right.\n");
    }
    md.append('\n');
  }

  static List<TagEstimator.Sighting> sightings(Inputs in, List<Frame> baseline) {
    List<TagEstimator.Sighting> out = new ArrayList<>();
    Map<String, Cameras.Camera> byName = new LinkedHashMap<>();
    in.cameras().forEach(c -> byName.put(c.name(), c));
    for (Frame f : baseline) {
      Pose2d t = in.truth().get(f.spot);
      Cameras.Camera c = byName.get(f.camera);
      if (t == null || c == null) {
        continue;
      }
      double[] m = c.llMount();
      var mount = CamGeom.mountFromLimelight(m[0], m[1], m[2], m[3], m[4], m[5]);
      for (Frame.Tag tag : f.tags) {
        Translation3d p = CamGeom.tagInField(new Pose3d(t), mount, CamGeom.tagInCamera(tag.txnc, tag.tync, tag.distM));
        out.add(new TagEstimator.Sighting(tag.id, f.camera, f.spot, p));
      }
    }
    return out;
  }

  private static void tags(StringBuilder md, Inputs in, List<Frame> baseline) {
    md.append("## 5. Are the tags where the layout says?\n\n");
    List<TagEstimator.Estimate> ests = TagEstimator.estimate(sightings(in, baseline), in.field().layout);
    if (ests.isEmpty()) {
      md.append("No tags seen.\n\n");
      return;
    }
    md.append(String.format(
        "Tag positions computed from the spots' truth poses and the configured mounts. Flagged when "
            + "more than %.0f cm from the layout. *Camera spread* = how far apart different cameras "
            + "put the same tag: small spread + big offset points at the tag; big spread points at a camera.%n%n",
        TagEstimator.misplacedM * 100));
    md.append("| Tag | Offset from layout | Camera spread | Sightings | Cameras | Spots | Confident |\n");
    md.append("|---|---|---|---|---|---|---|\n");
    for (TagEstimator.Estimate e : ests) {
      double off = e.offsetFromLayoutM();
      boolean flag = Double.isFinite(off) && off > TagEstimator.misplacedM;
      md.append(String.format(
          "| %d | %s | %.0f cm | %d | %s | %s | %s |%n",
          e.tagId(),
          Double.isFinite(off) ? (flag ? "**" : "") + String.format("%.0f cm", off * 100) + (flag ? "**" : "") : "not in layout",
          e.cameraSpreadM() * 100, e.sightings(), String.join(", ", e.cameras()),
          String.join(", ", e.spots()), e.confident() ? "yes" : "no"));
    }
    md.append("\n`fieldmap` writes a layout with the confident estimates.\n\n");
  }

  private static void sweep(StringBuilder md, Inputs in) {
    md.append("## 6. Exposure / gain — one setting per camera across all spots\n\n");
    List<SweepScorer.Choice> choices = choices(in);
    if (choices.isEmpty()) {
      md.append("No sweep in this run.\n\n");
      return;
    }
    md.append("Chosen = best worst-spot score, then best weighted average, then lower exposure (less blur). "
        + "Scores are 0–1 (visibility x steadiness x frame rate) relative to the best at that spot.\n\n");
    for (SweepScorer.Choice c : choices) {
      md.append("### ").append(c.camera()).append("\n\n");
      if (c.setting() == null) {
        md.append("Not tuned: saw too few tags at every spot (").append(String.join(", ", c.excludedSpots())).append(").\n\n");
        continue;
      }
      md.append(String.format("**Recommended: %s** (worst spot %.2f, weighted avg %.2f)%n%n",
          c.setting(), c.worstScore(), c.weightedMean()));
      md.append("| Spot | Weight | Score with recommended | Best setting at this spot alone |\n|---|---|---|---|\n");
      for (var e : c.scoreBySpot().entrySet()) {
        md.append(String.format("| %s | %.1f | %.2f | %s |%n", e.getKey(),
            in.weights().getOrDefault(e.getKey(), 1.0), e.getValue(), c.bestSettingBySpot().get(e.getKey())));
      }
      if (!c.excludedSpots().isEmpty()) {
        md.append("\nExcluded (camera barely saw tags there): ").append(String.join(", ", c.excludedSpots())).append('\n');
      }
      md.append("\nApply with `apply --camera ").append(c.camera()).append("` (asks before saving).\n\n");
    }
  }

  /** One recommended setting per camera, reconciled across the run's complete spots. */
  public static List<SweepScorer.Choice> choices(Inputs in) {
    List<Frame> sw = in.frames().stream()
        .filter(f -> f.phase.equals("sweep") && in.truth().containsKey(f.spot)).toList();
    if (sw.isEmpty()) {
      return List.of();
    }
    Map<String, Double> exp = new LinkedHashMap<>();
    Map<String, List<Frame>> byCam = new TreeMap<>();
    for (Frame f : sw) {
      byCam.computeIfAbsent(f.camera, k -> new ArrayList<>()).add(f);
    }
    List<SweepScorer.Cell> cells = new ArrayList<>();
    for (var e : byCam.entrySet()) {
      exp.clear();
      for (Frame f : e.getValue()) {
        exp.put(f.setting, in.exposureByCameraSetting().getOrDefault(f.camera + "|" + f.setting, Double.NaN));
      }
      cells.addAll(SweepScorer.cells(e.getValue(), exp));
    }
    Map<String, Double> baselineDetect = new LinkedHashMap<>();
    for (Frame f : in.frames()) {
      if (f.phase.equals("baseline")) {
        baselineDetect.merge(f.camera + "|" + f.spot, 0.0, (a, b) -> a);
      }
    }
    for (String key : baselineDetect.keySet()) {
      String[] k = key.split("\\|");
      List<Frame> fs = in.frames().stream()
          .filter(f -> f.phase.equals("baseline") && f.camera.equals(k[0]) && f.spot.equals(k[1])).toList();
      baselineDetect.put(key, fs.stream().filter(Frame::hasTags).count() / (double) Math.max(1, fs.size()));
    }
    return SweepScorer.reconcile(cells, in.weights(), baselineDetect);
  }

  private static void odometry(StringBuilder md, Inputs in) {
    if (in.odometry().isEmpty()) {
      return;
    }
    md.append("## 7. Robot odometry at each spot (for reference)\n\n");
    md.append("Where the robot's own pose estimate thought it was when each spot was marked.\n\n");
    for (var e : in.odometry().entrySet()) {
      Pose2d t = in.truth().get(e.getKey());
      Pose2d o = e.getValue();
      if (t == null) {
        continue;
      }
      md.append(String.format("- %s: %.0f cm off, heading %+.1f°%n", e.getKey(),
          t.getTranslation().getDistance(o.getTranslation()) * 100,
          MountCheck.wrap(o.getRotation().minus(t.getRotation()).getDegrees())));
    }
    md.append('\n');
  }

  /** Questions the user answered with {@code answer} ("y"/"n"/"skip") for a camera. */
  static List<String> answers(Inputs in, String camera, String answer) {
    List<String> out = new ArrayList<>();
    if (in.confirmations() == null) {
      return out;
    }
    for (JsonNode n : in.confirmations()) {
      if (n.path("camera").asText().equals(camera) && n.path("answer").asText().equals(answer)) {
        out.add(n.path("question").asText());
      }
    }
    return out;
  }

  static List<double[]> poses(List<Frame> frames, String cam, String spot, boolean mt1) {
    List<double[]> out = new ArrayList<>();
    for (Frame f : frames) {
      if (f.camera.equals(cam) && f.spot.equals(spot)) {
        double[] p = mt1 ? f.mt1 : f.mt2;
        if (p != null) {
          out.add(p);
        }
      }
    }
    return out;
  }

  static double[] medianPose(List<double[]> ps) {
    double[] m = new double[6];
    for (int i = 0; i < 6; i++) {
      final int k = i;
      m[i] = MountCheck.median(ps.stream().mapToDouble(p -> p[k]).toArray());
    }
    return m;
  }

  private static String fmt(double v) {
    return v == Math.rint(v) ? String.format("%.0f", v) : String.format("%.1f", v);
  }

  /** Loads report inputs from a run's session.json + frames. */
  public static Inputs load(RunStore run, FieldConfig field, List<Cameras.Camera> cameras) throws java.io.IOException {
    JsonNode s = run.readSession();
    Map<String, Pose2d> truth = new LinkedHashMap<>();
    Map<String, Double> weights = new LinkedHashMap<>();
    Map<String, Pose2d> odom = new LinkedHashMap<>();
    for (JsonNode sp : s.path("spotsRecorded")) {
      if (!sp.path("complete").asBoolean(false)) {
        continue;
      }
      String id = sp.path("id").asText();
      truth.put(id, new Pose2d(sp.path("truthX").asDouble(), sp.path("truthY").asDouble(),
          Rotation2d.fromDegrees(sp.path("truthHeadingDeg").asDouble())));
      weights.put(id, sp.path("weight").asDouble(1.0));
      if (sp.has("odomX")) {
        odom.put(id, new Pose2d(sp.path("odomX").asDouble(), sp.path("odomY").asDouble(),
            Rotation2d.fromDegrees(sp.path("odomHeadingDeg").asDouble())));
      }
    }
    Map<String, Double> exp = new LinkedHashMap<>();
    for (JsonNode g : s.path("grids")) {
      for (JsonNode set : g.path("settings")) {
        exp.put(g.path("camera").asText() + "|" + set.path("label").asText(), set.path("exposure").asDouble());
      }
    }
    Map<String, JsonNode> pipes = new LinkedHashMap<>();
    for (Cameras.Camera c : cameras) {
      java.io.File p = new java.io.File(run.backupDir(), c.name() + "-pipeline.json");
      if (p.exists()) {
        pipes.put(c.name(), RunStore.MAPPER.readTree(p));
      }
    }
    return new Inputs(field, cameras, truth, weights, odom, run.readFrames(), exp, pipes, s.path("confirmations"));
  }
}
