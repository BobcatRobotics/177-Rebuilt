package frc.vision.cal;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import frc.vision.cal.Cameras.Camera;
import frc.vision.cal.FieldConfig.Spot;
import frc.vision.cal.SettingsGrid.Setting;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

/**
 * Drives a calibration session: for each chosen spot, waits for the dashboard "Mark spot" button
 * (or Enter), records a baseline at the cameras' current settings, then optionally sweeps
 * exposure/gain on every camera in parallel and restores the original settings.
 */
public final class Session {
  public static double baselineSeconds = 3.0;
  public static double settleSeconds = 0.6;
  public static double sampleSecondsFast = 1.5;
  public static double sampleSecondsFull = 2.0;

  private final FieldConfig field;
  private final List<Spot> spots;
  private final List<Camera> cameras;
  private final RunStore run;
  private final ObjectNode session;
  private final Map<String, SettingsGrid> grids; // empty when not sweeping
  private final boolean fast;
  private volatile boolean restored = true;

  /** Ask "which field wall is each camera pointing at?" (first spot only unless askEverySpot). */
  public boolean askFieldDirections = true;

  public boolean askEverySpot = false;
  private boolean askedFieldOnce = false;

  private NetworkTableInstance nt;
  private BooleanEntry markSpot;
  private StringPublisher nextSpot;
  private StringPublisher status;
  private StructSubscriber<Pose2d> odometry;

  public Session(
      FieldConfig field,
      List<Spot> spots,
      List<Camera> cameras,
      RunStore run,
      ObjectNode session,
      Map<String, SettingsGrid> grids,
      boolean fast) {
    this.field = field;
    this.spots = spots;
    this.cameras = cameras;
    this.run = run;
    this.session = session;
    this.grids = grids;
    this.fast = fast;
  }

  public void connectNt(int team, String server) {
    nt = NetworkTableInstance.getDefault();
    nt.startClient4("limelightCal");
    if (server != null) {
      nt.setServer(server);
    } else {
      nt.setServerTeam(team);
    }
    var table = nt.getTable("VisionCal");
    markSpot = table.getBooleanTopic("MarkSpot").getEntry(false);
    markSpot.set(false);
    nextSpot = table.getStringTopic("NextSpot").publish();
    status = table.getStringTopic("Status").publish();
    odometry =
        nt.getStructTopic("/AdvantageKit/RealOutputs/Odometry/Robot", Pose2d.struct)
            .subscribe(new Pose2d());
  }

  public void run() throws IOException, InterruptedException {
    Thread restoreHook = new Thread(this::restoreAll, "restore-limelights");
    Runtime.getRuntime().addShutdownHook(restoreHook);
    ArrayNode doneSpots = session.putArray("spotsRecorded");
    try {
      for (int i = 0; i < spots.size(); i++) {
        Spot spot = spots.get(i);
        Pose2d truth = field.truthPose2d(spot);
        String where =
            String.format(
                "%s (%d/%d): %s  -> x=%.2f y=%.2f heading=%.0f°",
                spot.id, i + 1, spots.size(), spot.note.isEmpty() ? spot.type : spot.note,
                truth.getX(), truth.getY(), truth.getRotation().getDegrees());
        say("Drive to " + where);
        say("Park, then press Mark spot on the dashboard (or Enter here). 's' = skip, 'q' = quit.");
        publish("Drive to " + spot.id, "waiting");
        String action = waitForMark();
        if (action.equals("q")) {
          say("Stopping early.");
          break;
        }
        if (action.equals("s")) {
          say("Skipped " + spot.id);
          continue;
        }
        ObjectNode rec = doneSpots.addObject();
        rec.put("id", spot.id);
        rec.put("weight", spot.weight);
        rec.put("truthX", truth.getX());
        rec.put("truthY", truth.getY());
        rec.put("truthHeadingDeg", truth.getRotation().getDegrees());
        if (odometry != null && nt.isConnected()) {
          Pose2d o = odometry.get();
          rec.put("odomX", o.getX());
          rec.put("odomY", o.getY());
          rec.put("odomHeadingDeg", o.getRotation().getDegrees());
        }
        run.writeSession(session);

        if (askFieldDirections && (askEverySpot || !askedFieldOnce)) {
          askedFieldOnce = true;
          askFieldDirections(spot, truth);
        }

        publish(spot.id, "recording baseline");
        say("Recording baseline...");
        Map<String, Integer> counts = sample(spot.id, "baseline", "current", baselineSeconds, null);
        say("  frames with tags: " + counts);

        if (!grids.isEmpty()) {
          sweepAt(spot.id);
        }
        rec.put("complete", true);
        run.writeSession(session);
        say("Done with " + spot.id + ".");
      }
    } finally {
      restoreAll();
      publish("", "finished");
      try {
        Runtime.getRuntime().removeShutdownHook(restoreHook);
      } catch (IllegalStateException ignored) {
        // already shutting down
      }
    }
  }

  private void sweepAt(String spotId) throws IOException, InterruptedException {
    Map<String, List<Setting>> perCam = new LinkedHashMap<>();
    int steps = 0;
    for (var e : grids.entrySet()) {
      List<Setting> s = e.getValue().settings(fast);
      perCam.put(e.getKey(), s);
      steps = Math.max(steps, s.size());
    }
    double sampleS = fast ? sampleSecondsFast : sampleSecondsFull;
    say(String.format("Sweeping %d settings per camera (~%.0f s)...", steps, steps * (sampleS + settleSeconds)));
    restored = false;
    for (int k = 0; k < steps; k++) {
      Map<String, String> labels = new LinkedHashMap<>();
      for (Camera c : cameras) {
        List<Setting> s = perCam.get(c.name());
        if (s == null || k >= s.size()) {
          continue;
        }
        try {
          c.http().updatePipeline(s.get(k).values(), false);
          labels.put(c.name(), s.get(k).label());
        } catch (IOException ex) {
          say("  " + c.name() + ": could not apply setting (" + ex.getMessage() + ")");
        }
      }
      publish(spotId, "sweep " + (k + 1) + "/" + steps);
      Thread.sleep((long) (settleSeconds * 1000));
      sample(spotId, "sweep", null, sampleS, labels);
    }
    restoreAll();
  }

  /** Puts every swept camera back to its backed-up exposure/gain (never saved to disk). */
  synchronized void restoreAll() {
    if (restored) {
      return;
    }
    for (Camera c : cameras) {
      SettingsGrid g = grids.get(c.name());
      if (g == null) {
        continue;
      }
      try {
        c.http().updatePipeline(g.original, false);
      } catch (IOException ex) {
        System.err.println(
            "!! Could not restore "
                + c.name()
                + " ("
                + ex.getMessage()
                + "). Its settings were never saved, so power-cycling it restores them.");
      }
    }
    restored = true;
  }

  /** NetworkTables subscribers for one Limelight's documented output arrays. */
  private static final class LlTopics {
    final DoubleSubscriber hb;
    final DoubleArraySubscriber mt1;
    final DoubleArraySubscriber mt2;
    final DoubleArraySubscriber raw;
    final DoubleArraySubscriber hw;

    LlTopics(NetworkTableInstance nt, String name) {
      var t = nt.getTable(name);
      hb = t.getDoubleTopic("hb").subscribe(Double.NaN);
      mt1 = t.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);
      mt2 = t.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
      raw = t.getDoubleArrayTopic("rawfiducials").subscribe(new double[0]);
      hw = t.getDoubleArrayTopic("hw").subscribe(new double[0]);
    }
  }

  private final Map<String, LlTopics> topics = new LinkedHashMap<>();

  /** True if the camera's heartbeat is changing over NetworkTables. */
  private boolean ntLive(Camera c) throws InterruptedException {
    if (nt == null || !nt.isConnected()) {
      return false;
    }
    LlTopics t = topics.computeIfAbsent(c.name(), n -> new LlTopics(nt, n));
    double h0 = t.hb.get();
    for (int i = 0; i < 10; i++) {
      Thread.sleep(30);
      double h = t.hb.get();
      if (Double.isFinite(h) && h != h0) {
        return true;
      }
    }
    return false;
  }

  /**
   * Records every camera for {@code seconds}: over NetworkTables when the camera's heartbeat is
   * live (every frame, including ones with no tags), otherwise by polling /results. Returns frames
   * with tags per camera.
   */
  private Map<String, Integer> sample(
      String spotId, String phase, String fixedLabel, double seconds, Map<String, String> labels)
      throws InterruptedException {
    ExecutorService pool = Executors.newFixedThreadPool(cameras.size());
    Map<String, Future<Integer>> futures = new LinkedHashMap<>();
    long end = System.nanoTime() + (long) (seconds * 1e9);
    for (Camera c : cameras) {
      String label = fixedLabel != null ? fixedLabel : (labels == null ? null : labels.get(c.name()));
      if (label == null) {
        continue;
      }
      if (ntLive(c)) {
        LlTopics t = topics.get(c.name());
        futures.put(
            c.name(),
            pool.submit(
                () -> {
                  double lastHb = t.hb.get();
                  int withTags = 0;
                  while (System.nanoTime() < end) {
                    double h = t.hb.get();
                    if (h != lastHb) {
                      lastHb = h;
                      Frame f =
                          Frame.fromNt(c.name(), h * 1000.0, t.mt1.get(), t.mt2.get(), t.raw.get(), t.hw.get());
                      f.spot = spotId;
                      f.phase = phase;
                      f.setting = label;
                      run.append(f);
                      if (f.hasTags()) {
                        withTags++;
                      }
                    }
                    Thread.sleep(5);
                  }
                  return withTags;
                }));
        continue;
      }
      futures.put(
          c.name(),
          pool.submit(
              () -> {
                LimelightHttp http = c.http();
                double fps = fps(http);
                double lastTs = Double.NaN;
                int withTags = 0;
                while (System.nanoTime() < end) {
                  try {
                    Frame f = Frame.parse(http.results(), c.name());
                    if (!(f.llTimestampMs == lastTs)) {
                      lastTs = f.llTimestampMs;
                      f.spot = spotId;
                      f.phase = phase;
                      f.setting = label;
                      f.fps = fps;
                      run.append(f);
                      if (f.hasTags()) {
                        withTags++;
                      }
                    }
                  } catch (IOException ex) {
                    // Camera unreachable this poll; keep trying until the window ends
                  }
                  Thread.sleep(100);
                }
                return withTags;
              }));
    }
    Map<String, Integer> counts = new LinkedHashMap<>();
    for (var e : futures.entrySet()) {
      try {
        counts.put(e.getKey(), e.getValue().get());
      } catch (Exception ex) {
        counts.put(e.getKey(), -1);
      }
    }
    pool.shutdown();
    pool.awaitTermination(2, TimeUnit.SECONDS);
    return counts;
  }

  /** Frame rate from /status if the firmware reports one (key containing "fps"). */
  static double fps(LimelightHttp http) {
    try {
      JsonNode s = http.status();
      for (Iterator<String> it = s.fieldNames(); it.hasNext(); ) {
        String k = it.next();
        if (k.toLowerCase().contains("fps") && s.get(k).isNumber()) {
          return s.get(k).asDouble();
        }
      }
    } catch (IOException ignored) {
      // fall through
    }
    return Double.NaN;
  }

  /** Waits for the dashboard button or a line on stdin. Returns "mark", "s" or "q". */
  private String waitForMark() throws IOException, InterruptedException {
    while (true) {
      if (markSpot != null && markSpot.get()) {
        markSpot.set(false);
        return "mark";
      }
      if (Ask.ready()) {
        String l = Ask.line().toLowerCase();
        return l.equals("s") || l.equals("q") ? l : "mark";
      }
      Thread.sleep(50);
    }
  }

  /**
   * With the robot parked at a known heading, says which field wall each camera should be
   * pointing at (from VisionConstants yaw) and asks the user to confirm by looking.
   */
  private void askFieldDirections(Spot spot, Pose2d truth) throws IOException {
    ArrayNode out = session.withArray("confirmations");
    say(MountWords.FIELD_FRAME);
    say("Robot at " + spot.id + " should have its front (" + MountWords.frontWords(cameras)
        + ") facing " + MountWords.robotFacing(truth.getRotation()) + ".");
    for (Camera c : cameras) {
      String q =
          c.name() + " should be pointing at " + MountWords.fieldDirection(truth.getRotation(), c.llMount()[5])
              + ". Is it?";
      String ans = Ask.yesNo(q);
      ObjectNode n = out.addObject();
      n.put("camera", c.name());
      n.put("kind", "field");
      n.put("spot", spot.id);
      n.put("question", q);
      n.put("answer", ans);
      if (ans.equals("n")) {
        say("   -> noted: check " + c.name() + "'s yaw in VisionConstants (and that the robot is parked as described).");
      }
    }
    run.writeSession(session);
  }

  private void publish(String next, String state) {
    if (nextSpot != null) {
      nextSpot.set(next);
      status.set(state);
    }
  }

  static void say(String s) {
    System.out.println(s);
  }

  /** Camera names that have frames with tags in the baseline at a spot. */
  static List<String> camerasWithTags(List<Frame> frames, String spot) {
    List<String> out = new ArrayList<>();
    for (Frame f : frames) {
      if (f.spot.equals(spot) && f.phase.equals("baseline") && f.hasTags() && !out.contains(f.camera)) {
        out.add(f.camera);
      }
    }
    return out;
  }
}
