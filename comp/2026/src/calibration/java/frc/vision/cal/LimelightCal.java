package frc.vision.cal;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.JsonNodeFactory;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import frc.vision.cal.Cameras.Camera;
import frc.vision.cal.FieldConfig.Spot;
import frc.vision.cal.SettingsGrid.Setting;
import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Limelight field calibration tool. Runs on the laptop, tethered to the robot network.
 *
 * <pre>
 * gradlew limelightCal --console=plain -Pcal="&lt;command&gt; [options]"
 * </pre>
 *
 * See vision-cal/README.md for the field procedure.
 */
public final class LimelightCal {
  static final File RUNS = new File("vision-cal/runs");
  static final File PP_SETTINGS = new File("src/main/deploy/pathplanner/settings.json");

  public static void main(String[] argv) throws Exception {
    Args a = new Args(argv);
    try {
      switch (a.command) {
        case "check" -> check(a);
        case "backup" -> {
          try (RunStore run = RunStore.create(RUNS)) {
            ObjectNode s = JsonNodeFactory.instance.objectNode();
            backup(run, cameras(a), a, s);
            run.writeSession(s);
            System.out.println("Backup saved in " + run.dir);
          }
        }
        case "session" -> session(a);
        case "report" -> report(a);
        case "apply" -> apply(a);
        case "fieldmap" -> fieldmap(a);
        case "upload-fieldmap" -> uploadFieldmap(a);
        default -> help();
      }
    } catch (IOException | IllegalArgumentException e) {
      System.err.println("Error: " + e.getMessage());
      System.exit(1);
    }
    System.exit(0);
  }

  static void help() {
    System.out.println(
        """
        Limelight calibration tool. Usage: gradlew limelightCal --console=plain -Pcal="<command> [options]"

          check                         Static checks of VisionConstants mounts + can we reach each camera?
          backup                        Save every camera's pipeline settings (nothing changes on the cameras)
          session --field <file>        Record at each spot (dashboard Mark spot button or Enter), with
                  [--spots S1,S3]       exposure/gain sweep per spot unless --no-sweep. Sweeps are never
                  [--no-sweep] [--fast] saved to the cameras and are restored after every spot.
          report [--run <dir>]          Write report.md for a run (default: latest run)
          apply --camera <name>         Save the recommended exposure/gain to that camera (asks "yes" first)
          fieldmap [--run <dir>]        Estimate tag positions -> WPILib layout JSON + Limelight .fmap
          upload-fieldmap --file <fmap> Upload a field map to every camera (asks "yes" first)

        Direction questions: check/session read each camera's mount back in words and ask y/n;
        session also asks which field wall each camera points at (first spot, or --ask-every-spot).
        --no-ask skips them. check --offline --vpr-dir <dir> compares against saved .vpr files.

        Options: --cameras a,b  --hosts name=ip,...  --team 177  --server <ip>
                 --exposure-key exposure  --gain-key lcgain  --pipeline <index>
        """);
  }

  // ---------------------------------------------------------------- commands

  static void check(Args a) throws IOException {
    FieldConfig field = a.has("field") ? FieldConfig.load(new File(a.get("field")), PP_SETTINGS) : null;
    askRobotRelative(a, cameras(a), null);
    for (Camera c : cameras(a)) {
      System.out.println("== " + c.name() + " (" + c.host() + ")");
      double[] m = c.llMount();
      System.out.printf(
          "   mount: forward %.3f, side %.3f, up %.3f, roll %.1f, pitch %.1f, yaw %.1f%n",
          m[0], m[1], m[2], m[3], m[4], m[5]);
      MountCheck.staticNotes(m).forEach(n -> System.out.println("   ! " + n));
      if (field != null) {
        String o = MountCheck.outsideBumpers(m, field.robotLengthM, field.robotWidthM);
        if (o != null) {
          System.out.println("   ! " + o);
        }
      }
      if (a.has("vpr-dir")) {
        // Offline: compare against a saved pipeline file whose name contains the camera's suffix
        String suffix = c.name().replace("limelight-", "").toLowerCase();
        File[] vprs = new File(a.get("vpr-dir")).listFiles(
            f -> f.getName().toLowerCase().endsWith(".vpr") && f.getName().toLowerCase().contains(suffix));
        if (vprs == null || vprs.length == 0) {
          System.out.println("   (no .vpr for " + suffix + " in " + a.get("vpr-dir") + ")");
        } else {
          JsonNode pipe = RunStore.MAPPER.readTree(vprs[0]);
          List<String> diffs = MountCheck.compareToPipeline(m, MountCheck.pipelineMount(pipe));
          System.out.println("   vs " + vprs[0].getName() + ": " + (diffs.isEmpty() ? "mount matches" : ""));
          diffs.forEach(d -> System.out.println("   ! " + d));
        }
      }
      if (a.flag("offline")) {
        continue;
      }
      try {
        JsonNode st = c.http().status();
        System.out.println("   reachable; status: " + st);
        int idx = a.has("pipeline") ? Integer.parseInt(a.get("pipeline")) : activePipeline(c.http());
        JsonNode pipe = c.http().pipelineAtIndex(idx);
        for (String d : MountCheck.compareToPipeline(m, MountCheck.pipelineMount(pipe))) {
          System.out.println("   ! pose saved on the camera (pipeline " + idx + ") differs from VisionConstants: " + d);
        }
      } catch (IOException e) {
        System.out.println("   NOT reachable: " + e.getMessage());
      }
    }
  }

  /** Saves pipeline JSON, /status and /hwreport for each camera. Returns camera -> pipeline. */
  static Map<String, JsonNode> backup(RunStore run, List<Camera> cams, Args a, ObjectNode session)
      throws IOException {
    Map<String, JsonNode> pipes = new LinkedHashMap<>();
    ObjectNode b = session.putObject("backup");
    for (Camera c : cams) {
      LimelightHttp http = c.http();
      try {
        int index = a.has("pipeline") ? Integer.parseInt(a.get("pipeline")) : activePipeline(http);
        JsonNode pipe = http.pipelineAtIndex(index);
        run.writeJson(new File(run.backupDir(), c.name() + "-pipeline.json"), pipe);
        trySave(run, c.name() + "-status.json", http::status);
        trySave(run, c.name() + "-hwreport.json", http::hwreport);
        pipes.put(c.name(), pipe);
        b.putObject(c.name()).put("pipelineIndex", index);
        System.out.println("Backed up " + c.name() + " (pipeline " + index + ")");
      } catch (IOException e) {
        System.out.println("Could not back up " + c.name() + ": " + e.getMessage());
      }
    }
    return pipes;
  }

  static void session(Args a) throws Exception {
    if (!a.has("field")) {
      throw new IllegalArgumentException("session needs --field <file>");
    }
    File fieldFile = new File(a.get("field")).getAbsoluteFile();
    FieldConfig field = FieldConfig.load(fieldFile, PP_SETTINGS);
    List<Spot> spots = field.selectSpots(a.list("spots"));
    if (spots.isEmpty()) {
      throw new IllegalArgumentException("No spots selected (enable some in the field file or use --spots)");
    }
    List<Camera> cams = cameras(a);
    boolean sweep = !a.flag("no-sweep");
    boolean fast = a.flag("fast");

    try (RunStore run = RunStore.create(RUNS)) {
      ObjectNode s = JsonNodeFactory.instance.objectNode();
      s.put("fieldFile", fieldFile.getPath());
      s.put("fieldName", field.name);
      s.put("sweep", sweep);
      s.put("fast", fast);
      ArrayNode camNodes = s.putArray("cameras");
      for (Camera c : cams) {
        ObjectNode n = camNodes.addObject();
        n.put("name", c.name());
        n.put("host", c.host());
        ArrayNode m = n.putArray("llMount");
        for (double v : c.llMount()) {
          m.add(v);
        }
      }
      ArrayNode sel = s.putArray("spotsSelected");
      spots.forEach(sp -> sel.add(sp.id));

      Map<String, JsonNode> pipes = backup(run, cams, a, s);
      Map<String, SettingsGrid> grids = new LinkedHashMap<>();
      if (sweep) {
        ArrayNode gNodes = s.putArray("grids");
        for (Camera c : cams) {
          JsonNode pipe = pipes.get(c.name());
          if (pipe == null) {
            System.out.println("Not sweeping " + c.name() + " (no backup, so it could not be restored)");
            continue;
          }
          SettingsGrid g =
              new SettingsGrid(pipe, a.get("exposure-key", "exposure"), a.get("gain-key", "lcgain"));
          grids.put(c.name(), g);
          ObjectNode gn = gNodes.addObject();
          gn.put("camera", c.name());
          gn.set("original", g.original);
          ArrayNode sets = gn.putArray("settings");
          for (Setting st : g.settings(fast)) {
            ObjectNode sn = sets.addObject();
            sn.put("label", st.label());
            sn.put("exposure", st.exposure());
            sn.set("values", st.values());
          }
        }
      }
      run.writeSession(s);
      System.out.println("Run folder: " + run.dir);

      askRobotRelative(a, cams, s.putArray("confirmations"));
      run.writeSession(s);

      Session session = new Session(field, spots, cams, run, s, grids, fast);
      session.askFieldDirections = !a.flag("no-ask");
      session.askEverySpot = a.flag("ask-every-spot");
      session.connectNt(Integer.parseInt(a.get("team", "177")), a.get("server", null));
      System.out.println(
          "Dashboard: add /VisionCal/MarkSpot as a toggle button and /VisionCal/NextSpot as text in Elastic.");
      session.run();
      run.writeSession(s);
      System.out.println("Session done. Next: report --run " + run.dir);
    }
  }

  static void report(Args a) throws IOException {
    RunStore run = RunStore.open(runDir(a));
    Report.Inputs in = loadInputs(run);
    File out = new File(run.dir, "report.md");
    Files.writeString(out.toPath(), Report.build(in), StandardCharsets.UTF_8);
    System.out.println("Wrote " + out);
  }

  static void apply(Args a) throws IOException {
    String cam = a.get("camera", null);
    if (cam == null) {
      throw new IllegalArgumentException("apply needs --camera <name>");
    }
    RunStore run = RunStore.open(runDir(a));
    Report.Inputs in = loadInputs(run);
    SweepScorer.Choice choice =
        Report.choices(in).stream()
            .filter(c -> c.camera().equals(cam))
            .findFirst()
            .orElseThrow(() -> new IllegalArgumentException("No sweep result for " + cam));
    if (choice.setting() == null) {
      throw new IllegalArgumentException(cam + " had no usable spots, nothing to apply");
    }
    JsonNode values = null;
    JsonNode original = null;
    for (JsonNode g : run.readSession().path("grids")) {
      if (g.path("camera").asText().equals(cam)) {
        original = g.path("original");
        for (JsonNode st : g.path("settings")) {
          if (st.path("label").asText().equals(choice.setting())) {
            values = st.path("values");
          }
        }
      }
    }
    if (values == null) {
      throw new IOException("Setting " + choice.setting() + " not found in session.json");
    }
    Camera camera =
        cameraList(run).stream()
            .filter(c -> c.name().equals(cam))
            .findFirst()
            .orElseThrow(() -> new IllegalArgumentException("Unknown camera " + cam));
    System.out.println(cam + ": currently " + original + " -> recommended " + values);
    System.out.println("Spot scores with this setting: " + choice.scoreBySpot());
    if (!confirm("Save this to " + cam + "'s pipeline on the camera?")) {
      System.out.println("Not applied.");
      return;
    }
    camera.http().updatePipeline(values, true);
    System.out.println("Saved to " + cam + ". Backup of the old pipeline: " + run.backupDir());
  }

  static void fieldmap(Args a) throws IOException {
    RunStore run = RunStore.open(runDir(a));
    Report.Inputs in = loadInputs(run);
    List<Frame> baseline = in.frames().stream().filter(f -> f.phase.equals("baseline")).toList();
    List<TagEstimator.Estimate> ests =
        TagEstimator.estimate(Report.sightings(in, baseline), in.field().layout);
    AprilTagFieldLayout layout = TagEstimator.updatedLayout(in.field().layout, ests);
    String base = a.get("out", new File(run.dir, "fieldmap").getPath());
    File json = new File(base + ".json");
    File fmap = new File(base + ".fmap");
    layout.serialize(json.toPath());
    RunStore.MAPPER.writeValue(fmap, toFmap(layout, Double.parseDouble(a.get("tag-size-mm", "165.1"))));
    long used = ests.stream().filter(TagEstimator.Estimate::confident).count();
    System.out.printf("Updated %d of %d seen tags (others kept from %s).%n", used, ests.size(), in.field().layoutSource);
    for (TagEstimator.Estimate e : ests) {
      System.out.printf(
          "  tag %d: %s, %.0f cm from layout, camera spread %.0f cm%n",
          e.tagId(), e.confident() ? "updated" : "kept (low confidence)",
          e.offsetFromLayoutM() * 100, e.cameraSpreadM() * 100);
    }
    System.out.println("Wrote " + json + " (WPILib) and " + fmap + " (Limelight, field-center origin).");
  }

  static void uploadFieldmap(Args a) throws IOException {
    File f = new File(a.get("file", ""));
    if (!f.exists()) {
      throw new IllegalArgumentException("upload-fieldmap needs --file <path to .fmap>");
    }
    String body = Files.readString(f.toPath(), StandardCharsets.UTF_8);
    List<Camera> cams = cameras(a);
    System.out.println("Will upload " + f + " to: " + cams.stream().map(Camera::name).toList());
    if (!confirm("Replace the field map on these cameras?")) {
      System.out.println("Not uploaded.");
      return;
    }
    for (Camera c : cams) {
      try {
        c.http().uploadFieldmap(body);
        System.out.println("  uploaded to " + c.name());
      } catch (IOException e) {
        System.out.println("  FAILED for " + c.name() + ": " + e.getMessage());
      }
    }
    System.out.println("Verify: park at a known spot and check each camera's botpose before relying on it.");
  }

  // ---------------------------------------------------------------- helpers

  /** Limelight .fmap (field-center origin, row-major 4x4 transforms in meters, size in mm). */
  static ObjectNode toFmap(AprilTagFieldLayout layout, double sizeMm) {
    ObjectNode root = JsonNodeFactory.instance.objectNode();
    root.put("type", "frc");
    ArrayNode fids = root.putArray("fiducials");
    double cx = layout.getFieldLength() / 2;
    double cy = layout.getFieldWidth() / 2;
    for (AprilTag t : layout.getTags()) {
      Pose3d p = t.pose;
      var r = p.getRotation().toMatrix();
      ObjectNode f = fids.addObject();
      f.put("family", "apriltag3_36h11_classic");
      f.put("id", t.ID);
      f.put("size", sizeMm);
      ArrayNode m = f.putArray("transform");
      double[] trans = {p.getX() - cx, p.getY() - cy, p.getZ()};
      for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
          m.add(r.get(row, col));
        }
        m.add(trans[row]);
      }
      m.add(0).add(0).add(0).add(1);
      f.put("unique", 1);
    }
    return root;
  }

  static Report.Inputs loadInputs(RunStore run) throws IOException {
    JsonNode s = run.readSession();
    String fieldFile = s.path("fieldFile").asText("");
    if (fieldFile.isEmpty()) {
      throw new IOException("This run has no field (it was only a backup)");
    }
    FieldConfig field = FieldConfig.load(new File(fieldFile), PP_SETTINGS);
    return Report.load(run, field, cameraList(run));
  }

  /** Cameras and mounts exactly as recorded in the run (not today's VisionConstants). */
  static List<Camera> cameraList(RunStore run) throws IOException {
    List<Camera> out = new ArrayList<>();
    for (JsonNode c : run.readSession().path("cameras")) {
      double[] m = new double[6];
      for (int i = 0; i < 6; i++) {
        m[i] = c.path("llMount").path(i).asDouble();
      }
      out.add(new Camera(c.path("name").asText(), c.path("host").asText(), m));
    }
    return out;
  }

  static List<Camera> cameras(Args a) {
    Map<String, String> hosts = new LinkedHashMap<>();
    for (String kv : a.list("hosts")) {
      String[] p = kv.split("=", 2);
      if (p.length == 2) {
        hosts.put(p[0], p[1]);
      }
    }
    List<Camera> cams = Cameras.load(hosts, a.list("cameras"));
    if (cams.isEmpty()) {
      throw new IllegalArgumentException("No cameras matched --cameras " + a.list("cameras"));
    }
    return cams;
  }

  static int activePipeline(LimelightHttp http) {
    try {
      JsonNode r = http.results();
      JsonNode res = r.has("Results") ? r.get("Results") : r;
      if (res.has("pID")) {
        return res.get("pID").asInt();
      }
    } catch (IOException ignored) {
      // fall through
    }
    return 1; // VisionIOLimelight selects pipeline 1
  }

  interface Fetch {
    JsonNode get() throws IOException;
  }

  static void trySave(RunStore run, String name, Fetch f) {
    try {
      run.writeJson(new File(run.backupDir(), name), f.get());
    } catch (IOException ignored) {
      // optional
    }
  }

  static File runDir(Args a) throws IOException {
    return a.has("run") ? new File(a.get("run")) : RunStore.latest(RUNS);
  }

  static boolean confirm(String question) throws IOException {
    return Ask.confirm(question);
  }

  /**
   * Reads each camera's configured mount back in words and asks the user to confirm it against the
   * real robot. Answers are recorded (camera, kind "robot", question, answer) when {@code out} is
   * given. Skipped with --no-ask.
   */
  static void askRobotRelative(Args a, List<Camera> cams, ArrayNode out) throws IOException {
    if (a.flag("no-ask")) {
      return;
    }
    System.out.println();
    System.out.println("Look at the robot and confirm where each camera is (VisionConstants values, in words).");
    System.out.println(MountWords.frame(cams));
    System.out.println();
    for (Camera c : cams) {
      String q = c.name() + ": " + MountWords.robotRelative(c.llMount()) + ". Correct?";
      String ans = Ask.yesNo(q);
      if (ans.equals("n")) {
        System.out.println("   -> noted. Fix VisionConstants from CAD before trusting this camera's results.");
      }
      if (out != null) {
        ObjectNode n = out.addObject();
        n.put("camera", c.name());
        n.put("kind", "robot");
        n.put("question", q);
        n.put("answer", ans);
      }
    }
    System.out.println();
  }

  /** Tiny "--key value" / "--flag" parser. */
  static final class Args {
    final String command;
    private final Map<String, String> opts = new LinkedHashMap<>();

    Args(String[] argv) {
      List<String> l = new ArrayList<>(Arrays.asList(argv));
      l.removeIf(String::isBlank);
      command = l.isEmpty() ? "help" : l.remove(0);
      for (int i = 0; i < l.size(); i++) {
        String k = l.get(i);
        if (!k.startsWith("--")) {
          throw new IllegalArgumentException("Unexpected argument " + k);
        }
        k = k.substring(2);
        if (i + 1 < l.size() && !l.get(i + 1).startsWith("--")) {
          opts.put(k, l.get(++i));
        } else {
          opts.put(k, "true");
        }
      }
    }

    boolean has(String k) {
      return opts.containsKey(k);
    }

    boolean flag(String k) {
      return "true".equals(opts.get(k));
    }

    String get(String k) {
      return opts.get(k);
    }

    String get(String k, String def) {
      return opts.getOrDefault(k, def);
    }

    List<String> list(String k) {
      String v = opts.get(k);
      return v == null ? List.of() : Arrays.stream(v.split(",")).map(String::trim).filter(s -> !s.isEmpty()).toList();
    }
  }
}
