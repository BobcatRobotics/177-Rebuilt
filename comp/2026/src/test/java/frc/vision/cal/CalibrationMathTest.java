package frc.vision.cal;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.vision.cal.FieldConfig.RobotSide;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class CalibrationMathTest {
  // A small fake field: tags on two walls facing into the field
  static final AprilTagFieldLayout LAYOUT =
      new AprilTagFieldLayout(
          List.of(
              new AprilTag(1, new Pose3d(0.0, 2.0, 1.0, new Rotation3d(0, 0, 0))),
              new AprilTag(2, new Pose3d(0.0, 4.0, 1.2, new Rotation3d(0, 0, 0))),
              new AprilTag(3, new Pose3d(0.0, 6.0, 0.8, new Rotation3d(0, 0, 0))),
              new AprilTag(4, new Pose3d(8.0, 3.0, 1.0, new Rotation3d(0, 0, Math.PI))),
              new AprilTag(5, new Pose3d(8.0, 5.0, 1.1, new Rotation3d(0, 0, Math.PI)))),
          8.0,
          8.0);

  static final List<Pose2d> SPOTS =
      List.of(
          new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(180)),
          new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(160)),
          new Pose2d(5.5, 4.0, Rotation2d.fromDegrees(10)),
          new Pose2d(4.0, 2.5, Rotation2d.fromDegrees(200)));

  /** Detections a camera with mount {@code trueLl} would really make from every spot. */
  static List<MountCheck.Obs> observe(double[] trueLl, AprilTagFieldLayout layout, double noiseM) {
    Transform3d mount = CamGeom.mountFromLimelight(trueLl[0], trueLl[1], trueLl[2], trueLl[3], trueLl[4], trueLl[5]);
    List<MountCheck.Obs> out = new ArrayList<>();
    java.util.Random rnd = new java.util.Random(177);
    for (int s = 0; s < SPOTS.size(); s++) {
      Pose3d robot = new Pose3d(SPOTS.get(s));
      Pose3d cam = robot.transformBy(mount);
      for (AprilTag t : LAYOUT.getTags()) {
        Translation3d real = layout.getTagPose(t.ID).orElseThrow().getTranslation();
        Translation3d inCam = new Pose3d(real, Rotation3d.kZero).relativeTo(cam).getTranslation();
        if (inCam.getX() <= 0.3) {
          continue; // behind the camera
        }
        // Round-trip through what a Limelight reports (angles + distance)
        double[] d = CamGeom.detectionFromCamera(inCam);
        Translation3d seen = CamGeom.tagInCamera(d[0], d[1], d[2] + noiseM * rnd.nextGaussian());
        out.add(new MountCheck.Obs(robot, seen, LAYOUT.getTagPose(t.ID).orElseThrow().getTranslation(), t.ID, "S" + s));
      }
    }
    return out;
  }

  @Test
  void tagRelativeSpotFront() {
    Pose2d tag = new Pose2d(5, 4, Rotation2d.kZero); // tag faces +x
    Pose2d r = SpotMath.tagRelative(tag, RobotSide.FRONT, 0.0, 0.0, 1.0, 0.8);
    assertEquals(5.5, r.getX(), 1e-9);
    assertEquals(4.0, r.getY(), 1e-9);
    assertEquals(180, Math.abs(r.getRotation().getDegrees()), 1e-9);
  }

  @Test
  void tagRelativeSpotLeftSideWithGapAndShift() {
    Pose2d tag = new Pose2d(5, 4, Rotation2d.kZero);
    Pose2d r = SpotMath.tagRelative(tag, RobotSide.LEFT, 0.1, 0.5, 1.0, 0.8);
    assertEquals(5.5, r.getX(), 1e-9); // 0.1 gap + 0.4 half width
    // Looking at the tag (facing -x), "tag's left as seen from the field" is -y
    assertEquals(3.5, r.getY(), 1e-9);
    // Robot's left side must point at the tag (-x)
    Rotation2d left = r.getRotation().plus(Rotation2d.kCCW_90deg);
    assertEquals(-1.0, left.getCos(), 1e-9);
  }

  @Test
  void detectionRoundTrip() {
    Translation3d p = new Translation3d(3.0, -0.7, 0.4);
    double[] d = CamGeom.detectionFromCamera(p);
    assertTrue(d[0] > 0, "tag to the right gives positive txnc");
    assertTrue(d[1] > 0, "tag above gives positive tync");
    Translation3d back = CamGeom.tagInCamera(d[0], d[1], d[2]);
    assertEquals(0, back.getDistance(p), 1e-9);
  }

  @Test
  void mountConventionRoundTrip() {
    double[] ll = {0.26, -0.31, 0.17, 0, 27, 90};
    double[] back =
        CamGeom.mountToLimelight(CamGeom.mountFromLimelight(ll[0], ll[1], ll[2], ll[3], ll[4], ll[5]));
    for (int i = 0; i < 6; i++) {
      assertEquals(ll[i], back[i], 1e-9);
    }
    // side is right-positive in Limelight terms -> negative side is on the robot's left (+y)
    assertTrue(CamGeom.mountFromLimelight(ll[0], ll[1], ll[2], ll[3], ll[4], ll[5]).getY() > 0);
  }

  @Test
  void correctMountIsOk() {
    double[] ll = {-0.43, -0.25, 0.48, 0, 26, 180};
    MountCheck.Result r = MountCheck.check(ll, observe(ll, LAYOUT, 0.005));
    assertEquals(MountCheck.Verdict.OK, r.verdict());
    assertTrue(r.betterFits().isEmpty());
  }

  @Test
  void moderateErrorIsSuspiciousNotWrong() {
    double[] truth = {-0.43, -0.25, 0.48, 0, 26, 180};
    double[] configured = truth.clone();
    configured[0] += 0.15;
    configured[5] += 4;
    MountCheck.Result r = MountCheck.check(configured, observe(truth, LAYOUT, 0.005));
    assertEquals(MountCheck.Verdict.SUSPICIOUS, r.verdict());
  }

  @Test
  void flippedYawIsCalledOut() {
    double[] truth = {0.26, -0.31, 0.17, 0, 27, 90};
    double[] configured = truth.clone();
    configured[5] = -90;
    MountCheck.Result r = MountCheck.check(configured, observe(truth, LAYOUT, 0.005));
    assertEquals(MountCheck.Verdict.WRONG, r.verdict());
    assertFalse(r.betterFits().isEmpty());
    String best = r.betterFits().get(0).description();
    assertTrue(best.contains("yaw"), best);
  }

  @Test
  void inchesAsMetersIsCalledOut() {
    double[] truth = {-0.434, -0.25, 0.483, 0, 26, 180};
    double[] configured = {-0.434, -9.825, 19.03, 0, 26, 180}; // side/up typed in inches
    MountCheck.Result r = MountCheck.check(configured, observe(truth, LAYOUT, 0.005));
    assertEquals(MountCheck.Verdict.WRONG, r.verdict());
    assertTrue(r.betterFits().stream().anyMatch(h -> h.description().contains("inches")));
    assertFalse(MountCheck.staticNotes(configured).isEmpty());
  }

  @Test
  void misplacedTagIsFoundAndCamerasAgree() {
    // Tag 2 is really 20 cm further along y than the layout says
    List<AprilTag> real = new ArrayList<>(LAYOUT.getTags());
    real.replaceAll(t -> t.ID == 2 ? new AprilTag(2, new Pose3d(0.0, 4.2, 1.2, Rotation3d.kZero)) : t);
    AprilTagFieldLayout realLayout = new AprilTagFieldLayout(real, 8, 8);

    double[] camA = {-0.43, -0.25, 0.48, 0, 26, 180};
    double[] camB = {-0.40, 0.25, 0.45, 0, 20, 170};
    List<TagEstimator.Sighting> s = new ArrayList<>();
    for (double[] ll : List.of(camA, camB)) {
      Transform3d m = CamGeom.mountFromLimelight(ll[0], ll[1], ll[2], ll[3], ll[4], ll[5]);
      for (MountCheck.Obs o : observe(ll, realLayout, 0.0)) {
        for (int k = 0; k < 3; k++) { // a few frames each
          s.add(new TagEstimator.Sighting(o.tagId(), ll == camA ? "A" : "B", o.spot(),
              CamGeom.tagInField(o.robot(), m, o.tagInCam())));
        }
      }
    }
    var ests = TagEstimator.estimate(s, LAYOUT);
    TagEstimator.Estimate t2 = ests.stream().filter(e -> e.tagId() == 2).findFirst().orElseThrow();
    assertEquals(0.20, t2.offsetFromLayoutM(), 0.01);
    assertTrue(t2.cameraSpreadM() < 0.01, "both cameras agree, so the tag moved, not a camera");
    TagEstimator.Estimate t1 = ests.stream().filter(e -> e.tagId() == 1).findFirst().orElseThrow();
    assertTrue(t1.offsetFromLayoutM() < 0.01);
  }

  @Test
  void sweepPicksOneSettingThatWorksAtEverySpot() {
    // "hiExp" is great at S1 but poor at S2; "mid" is decent everywhere
    List<SweepScorer.Cell> cells = List.of(
        cell("S1", "hiExp", 2.0, 0.95), cell("S2", "hiExp", 2.0, 0.30),
        cell("S1", "mid", 1.0, 0.80), cell("S2", "mid", 1.0, 0.75),
        cell("S1", "lo", 0.5, 0.40), cell("S2", "lo", 0.5, 0.70));
    var c = SweepScorer.reconcile(cells, Map.of("S1", 1.0, "S2", 1.0), Map.of()).get(0);
    assertEquals("mid", c.setting());
    assertEquals("hiExp", c.bestSettingBySpot().get("S1"));
  }

  @Test
  void sweepPrefersLowerExposureWhenNearlyAsGood() {
    List<SweepScorer.Cell> cells = List.of(
        cell("S1", "a", 2.0, 0.90), cell("S1", "b", 1.0, 0.88));
    var c = SweepScorer.reconcile(cells, Map.of(), Map.of()).get(0);
    assertEquals("b", c.setting());
  }

  @Test
  void spotWhereCameraSeesNothingIsExcluded() {
    List<SweepScorer.Cell> cells = List.of(
        cell("S1", "a", 1.0, 0.9), cell("S2", "a", 1.0, 0.0),
        cell("S1", "b", 2.0, 0.5), cell("S2", "b", 2.0, 0.1));
    var c = SweepScorer.reconcile(cells, Map.of(), Map.of("cam|S2", 0.0)).get(0);
    assertEquals("a", c.setting());
    assertEquals(List.of("S2"), c.excludedSpots());
  }

  @Test
  void fmapRoundTrip() throws Exception {
    var fmap = LimelightCal.toFmap(LAYOUT, 165.1);
    AprilTagFieldLayout back = FieldConfig.fromFmap(fmap, 8.0, 8.0);
    for (AprilTag t : LAYOUT.getTags()) {
      Pose3d b = back.getTagPose(t.ID).orElseThrow();
      assertEquals(0, b.getTranslation().getDistance(t.pose.getTranslation()), 1e-9);
      assertEquals(t.pose.getRotation().getZ(), b.getRotation().getZ(), 1e-9);
    }
    assertNotNull(fmap.get("fiducials"));
  }

  @Test
  void savedPipelinePoseMismatchIsReported() throws Exception {
    var pipe = new com.fasterxml.jackson.databind.ObjectMapper().readTree(
        "{\"rsf\":0.1838,\"rss\":0.2423,\"rsu\":0.4925,\"rsroll\":0,\"rspitch\":26,\"rsyaw\":180}");
    double[] code = {-0.434, -0.25, 0.483, 0, 26, 180};
    List<String> d = MountCheck.compareToPipeline(code, MountCheck.pipelineMount(pipe));
    assertEquals(2, d.size(), d.toString()); // forward and side; up within 2 cm
    assertTrue(d.stream().allMatch(s -> s.contains("opposite sign")));
    assertTrue(MountCheck.compareToPipeline(code, MountCheck.pipelineMount(
        new com.fasterxml.jackson.databind.ObjectMapper().readTree(
            "{\"rsf\":-0.434,\"rss\":-0.25,\"rsu\":0.483,\"rsroll\":0,\"rspitch\":26,\"rsyaw\":180}"))).isEmpty());
  }

  @Test
  void mountsInWords() {
    // Intake per VisionConstants: back-left, pointing backward, tilted up
    String intake = MountWords.robotRelative(new double[] {-0.434, -0.25, 0.483, 0, 26, 180});
    assertTrue(intake.contains("0.43 m (17.1 in) back"), intake);
    assertTrue(intake.contains("0.25 m (9.8 in) left"), intake);
    assertTrue(intake.contains("pointing backward"), intake);
    assertTrue(intake.contains("26° up"), intake);
    assertEquals("left", MountWords.robotDirection(90));
    assertEquals("right", MountWords.robotDirection(-90));
  }

  @Test
  void fieldDirectionsFollowRobotHeading() {
    // Robot facing the red wall (+x): intake (yaw 180) points at the blue wall
    assertEquals("the blue alliance wall", MountWords.fieldDirection(Rotation2d.kZero, 180));
    // Robot facing the blue wall: fleft (yaw +90, robot's left) points at the right side wall
    assertTrue(MountWords.fieldDirection(Rotation2d.k180deg, 90).startsWith("the right side wall"));
    assertEquals("the red alliance wall", MountWords.robotFacing(Rotation2d.kZero));
  }

  @Test
  void official2026LayoutsLoad() throws Exception {
    for (String v : List.of("welded", "andymark")) {
      AprilTagFieldLayout l =
          FieldConfig.loadLayout("2026-rebuilt-" + v + ".json", new java.io.File("vision-cal/fields"), 0, 0);
      assertEquals(32, l.getTags().size(), v);
      // Tag 1 per WPILib's 2026 CSV: 35 in high, facing 180°
      Pose3d t1 = l.getTagPose(1).orElseThrow();
      assertEquals(0.889, t1.getZ(), 1e-3);
      assertEquals(180, Math.abs(Math.toDegrees(t1.getRotation().getZ())), 1e-6);
    }
  }

  static SweepScorer.Cell cell(String spot, String setting, double exposure, double score) {
    return new SweepScorer.Cell("cam", spot, setting, exposure, 20, 1, 2, 0.01, 60, Double.NaN, score);
  }
}
