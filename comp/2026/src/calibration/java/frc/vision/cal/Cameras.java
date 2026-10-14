package frc.vision.cal;

import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.limelightConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * The robot's Limelights and their configured mounts, read straight from
 * {@link VisionConstants#cameraConstants} so the tool always checks what the robot actually uses.
 */
public final class Cameras {
  public record Camera(String name, String host, double[] llMount) {
    public LimelightHttp http() {
      return new LimelightHttp(name, host);
    }
  }

  /**
   * @param hostOverrides name -> host/IP (default "name.local")
   * @param only if non-empty, just these camera names
   */
  public static List<Camera> load(Map<String, String> hostOverrides, List<String> only) {
    List<Camera> out = new ArrayList<>();
    for (limelightConstants c : VisionConstants.cameraConstants) {
      if (!only.isEmpty() && !only.contains(c.name)) {
        continue;
      }
      out.add(
          new Camera(
              c.name,
              hostOverrides.getOrDefault(c.name, c.name + ".local"),
              new double[] {c.forward, c.side, c.up, c.roll, c.pitch, c.yaw}));
    }
    return out;
  }
}
