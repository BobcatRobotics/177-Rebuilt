package frc.vision.cal;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.JsonNodeFactory;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Exposure x gain values to try for one camera, built around that camera's own current settings
 * (each camera is tuned independently).
 */
public final class SettingsGrid {
  public record Setting(String label, ObjectNode values, double exposure) {}

  // Multipliers around the current value. Index 0 of each is the current value so the baseline is
  // always part of the comparison.
  static final double[] FAST_EXP = {1.0, 0.5, 0.75, 1.5};
  static final double[] FAST_GAIN = {1.0, 1.5};
  static final double[] FULL_EXP = {1.0, 0.35, 0.5, 0.7, 1.4, 2.0};
  static final double[] FULL_GAIN = {1.0, 0.7, 1.4};

  public final String exposureKey;
  public final String gainKey;
  public final ObjectNode original; // just the keys we change, as they were

  public SettingsGrid(JsonNode pipeline, String exposureKey, String gainKey) throws IOException {
    this.exposureKey = exposureKey;
    this.gainKey = gainKey;
    if (!pipeline.has(exposureKey) || !pipeline.has(gainKey)) {
      throw new IOException(
          "Pipeline has no \""
              + (pipeline.has(exposureKey) ? gainKey : exposureKey)
              + "\" setting. Keys that look related: "
              + related(pipeline)
              + ". Re-run with --exposure-key / --gain-key.");
    }
    original = JsonNodeFactory.instance.objectNode();
    original.set(exposureKey, pipeline.get(exposureKey));
    original.set(gainKey, pipeline.get(gainKey));
  }

  public List<Setting> settings(boolean fast) {
    double[] em = fast ? FAST_EXP : FULL_EXP;
    double[] gm = fast ? FAST_GAIN : FULL_GAIN;
    double e0 = original.get(exposureKey).asDouble();
    double g0 = original.get(gainKey).asDouble();
    boolean eInt = original.get(exposureKey).isIntegralNumber();
    boolean gInt = original.get(gainKey).isIntegralNumber();
    List<Setting> out = new ArrayList<>();
    for (double gmul : gm) {
      for (double emul : em) {
        double e = Math.max(eInt ? 1 : 0.01, e0 * emul);
        double g = Math.max(eInt ? 0 : 0.0, g0 * gmul);
        ObjectNode v = JsonNodeFactory.instance.objectNode();
        if (eInt) {
          v.put(exposureKey, Math.round(e));
        } else {
          v.put(exposureKey, e);
        }
        if (gInt) {
          v.put(gainKey, Math.round(g));
        } else {
          v.put(gainKey, g);
        }
        String label =
            (emul == 1.0 && gmul == 1.0)
                ? "current"
                : String.format("%s=%s,%s=%s", exposureKey, v.get(exposureKey), gainKey, v.get(gainKey));
        out.add(new Setting(label, v, v.get(exposureKey).asDouble()));
      }
    }
    return out;
  }

  private static String related(JsonNode pipeline) {
    List<String> keys = new ArrayList<>();
    for (Iterator<String> it = pipeline.fieldNames(); it.hasNext(); ) {
      String k = it.next();
      String l = k.toLowerCase();
      if (l.contains("expos") || l.contains("gain") || l.contains("black")) {
        keys.add(k);
      }
    }
    return keys.isEmpty() ? "(none)" : String.join(", ", keys);
  }
}
