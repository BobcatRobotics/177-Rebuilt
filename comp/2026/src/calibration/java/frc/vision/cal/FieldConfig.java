package frc.vision.cal;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

/**
 * A field definition: which AprilTag layout to use and which spots the robot can be parked at.
 * Game-agnostic; see vision-cal/README.md for the file format.
 */
public final class FieldConfig {
  public enum RobotSide {
    FRONT,
    BACK,
    LEFT,
    RIGHT
  }

  /** A place the robot is parked during calibration. */
  public static final class Spot {
    public String id;
    public String type; // "tagRelative" or "absolute"
    public int tag;
    public double faceGapM;
    public double lateralM;
    public RobotSide touching = RobotSide.FRONT;
    public double x;
    public double y;
    public double headingDeg;
    public double weight = 1.0;
    public boolean enabled = true;
    public String note = "";
  }

  public final String name;
  public final AprilTagFieldLayout layout;
  public final String layoutSource;
  public final double robotLengthM;
  public final double robotWidthM;
  public final List<Spot> spots;

  private FieldConfig(
      String name,
      AprilTagFieldLayout layout,
      String layoutSource,
      double robotLengthM,
      double robotWidthM,
      List<Spot> spots) {
    this.name = name;
    this.layout = layout;
    this.layoutSource = layoutSource;
    this.robotLengthM = robotLengthM;
    this.robotWidthM = robotWidthM;
    this.spots = spots;
  }

  private static final ObjectMapper MAPPER = new ObjectMapper();

  /**
   * Loads a field file. Robot bumper size defaults to PathPlanner's settings.json (robotLength /
   * robotWidth) unless the file sets robotLengthM / robotWidthM.
   */
  public static FieldConfig load(File file, File pathplannerSettings) throws IOException {
    JsonNode root = MAPPER.readTree(file);
    String name = root.path("name").asText(file.getName());

    double length = 0;
    double width = 0;
    if (pathplannerSettings != null && pathplannerSettings.exists()) {
      JsonNode pp = MAPPER.readTree(pathplannerSettings);
      length = pp.path("robotLength").asDouble(0);
      width = pp.path("robotWidth").asDouble(0);
    }
    length = root.path("robotLengthM").asDouble(length);
    width = root.path("robotWidthM").asDouble(width);
    if (length <= 0 || width <= 0) {
      throw new IOException("Robot size unknown: set robotLengthM/robotWidthM in " + file);
    }

    String layoutSpec = root.path("layout").asText("");
    AprilTagFieldLayout layout =
        loadLayout(
            layoutSpec,
            file.getParentFile(),
            root.path("fieldLengthM").asDouble(0),
            root.path("fieldWidthM").asDouble(0));

    List<Spot> spots = new ArrayList<>();
    Set<String> ids = new LinkedHashSet<>();
    for (JsonNode s : root.path("spots")) {
      Spot spot = new Spot();
      spot.id = s.path("id").asText();
      if (spot.id.isEmpty() || !ids.add(spot.id)) {
        throw new IOException("Every spot needs a unique id (problem near " + s + ")");
      }
      spot.type = s.path("type").asText("absolute");
      spot.tag = s.path("tag").asInt(-1);
      spot.faceGapM = s.path("faceGapM").asDouble(0);
      spot.lateralM = s.path("lateralM").asDouble(0);
      spot.touching = RobotSide.valueOf(s.path("touching").asText("front").toUpperCase());
      spot.x = s.path("x").asDouble(0);
      spot.y = s.path("y").asDouble(0);
      spot.headingDeg = s.path("headingDeg").asDouble(0);
      spot.weight = s.path("weight").asDouble(1.0);
      spot.enabled = s.path("enabled").asBoolean(true);
      spot.note = s.path("note").asText("");
      if (spot.type.equals("tagRelative") && layout.getTagPose(spot.tag).isEmpty()) {
        throw new IOException("Spot " + spot.id + " references tag " + spot.tag + " not in layout");
      }
      spots.add(spot);
    }
    return new FieldConfig(name, layout, layoutSpec, length, width, spots);
  }

  /** Spots to calibrate: the --spots list if given, otherwise every enabled spot, in file order. */
  public List<Spot> selectSpots(List<String> requested) {
    if (requested == null || requested.isEmpty()) {
      return spots.stream().filter(s -> s.enabled).toList();
    }
    List<Spot> out = new ArrayList<>();
    for (String id : requested) {
      out.add(
          spots.stream()
              .filter(s -> s.id.equals(id))
              .findFirst()
              .orElseThrow(() -> new IllegalArgumentException("No spot named " + id)));
    }
    return out;
  }

  /** True robot pose (on the floor) for a spot. */
  public Pose3d truthPose(Spot spot) {
    return new Pose3d(truthPose2d(spot));
  }

  public Pose2d truthPose2d(Spot spot) {
    if (!spot.type.equals("tagRelative")) {
      return new Pose2d(spot.x, spot.y, Rotation2d.fromDegrees(spot.headingDeg));
    }
    return SpotMath.tagRelative(
        layout.getTagPose(spot.tag).orElseThrow().toPose2d(),
        spot.touching,
        spot.faceGapM,
        spot.lateralM,
        robotLengthM,
        robotWidthM);
  }

  /**
   * Loads a layout from a WPILib AprilTagFields name (e.g. "k2025ReefscapeWelded"), a WPILib
   * layout JSON, or a Limelight .fmap (center-origin; needs fieldLengthM/fieldWidthM).
   */
  public static AprilTagFieldLayout loadLayout(
      String spec, File baseDir, double fieldLengthM, double fieldWidthM) throws IOException {
    if (spec.isEmpty()) {
      throw new IOException("Field file needs a \"layout\"");
    }
    if (spec.startsWith("k")) {
      try {
        return AprilTagFieldLayout.loadField(AprilTagFields.valueOf(spec));
      } catch (IllegalArgumentException e) {
        throw new IOException(
            "Unknown AprilTagFields name " + spec + "; use a layout JSON or .fmap path instead");
      }
    }
    File f = new File(spec);
    if (!f.isAbsolute() && baseDir != null) {
      f = new File(baseDir, spec);
    }
    if (spec.toLowerCase().endsWith(".fmap")) {
      return fromFmap(MAPPER.readTree(f), fieldLengthM, fieldWidthM);
    }
    return new AprilTagFieldLayout(f.toPath());
  }

  /** Limelight .fmap uses field-center origin; WPILib layouts use the blue-corner origin. */
  static AprilTagFieldLayout fromFmap(JsonNode fmap, double lengthM, double widthM)
      throws IOException {
    if (lengthM <= 0 || widthM <= 0) {
      throw new IOException("An .fmap layout needs fieldLengthM and fieldWidthM in the field file");
    }
    List<AprilTag> tags = new ArrayList<>();
    for (JsonNode f : fmap.path("fiducials")) {
      double[] m = new double[16];
      for (int i = 0; i < 16; i++) {
        m[i] = f.path("transform").path(i).asDouble();
      }
      tags.add(new AprilTag(f.path("id").asInt(), poseFromRowMajor(m, lengthM / 2, widthM / 2)));
    }
    return new AprilTagFieldLayout(tags, lengthM, widthM);
  }

  static Pose3d poseFromRowMajor(double[] m, double shiftX, double shiftY) {
    var rot =
        new edu.wpi.first.math.Matrix<>(edu.wpi.first.math.Nat.N3(), edu.wpi.first.math.Nat.N3());
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        rot.set(r, c, m[r * 4 + c]);
      }
    }
    return new Pose3d(
        new Translation3d(m[3] + shiftX, m[7] + shiftY, m[11]), new Rotation3d(rot));
  }

  /** Tag field position, if the layout has it. */
  public Optional<Translation3d> tagPosition(int id) {
    return layout.getTagPose(id).map(Pose3d::getTranslation);
  }

  static Translation2d unit(Rotation2d r) {
    return new Translation2d(r.getCos(), r.getSin());
  }
}
