package frc.vision.cal;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.StandardOpenOption;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

/**
 * A run folder: vision-cal/runs/<timestamp>/ with session.json (what was done), frames.jsonl (every
 * Limelight frame) and backup/ (camera settings before anything changed).
 */
public final class RunStore implements AutoCloseable {
  static final ObjectMapper MAPPER =
      new ObjectMapper()
          .enable(SerializationFeature.INDENT_OUTPUT)
          .disable(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES);
  private static final ObjectMapper LINE = new ObjectMapper();

  public final File dir;
  private BufferedWriter frames;

  private RunStore(File dir) {
    this.dir = dir;
  }

  public static RunStore create(File root) throws IOException {
    String ts = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss"));
    File d = new File(root, ts);
    if (!d.mkdirs()) {
      throw new IOException("Could not create " + d);
    }
    return new RunStore(d);
  }

  public static RunStore open(File dir) throws IOException {
    if (!new File(dir, "session.json").exists()) {
      throw new IOException(dir + " is not a calibration run (no session.json)");
    }
    return new RunStore(dir);
  }

  /** Most recent run under root, for commands where --run is omitted. */
  public static File latest(File root) throws IOException {
    File[] runs = root.listFiles(f -> new File(f, "session.json").exists());
    if (runs == null || runs.length == 0) {
      throw new IOException("No runs in " + root);
    }
    File best = runs[0];
    for (File f : runs) {
      if (f.getName().compareTo(best.getName()) > 0) {
        best = f;
      }
    }
    return best;
  }

  public File backupDir() {
    File b = new File(dir, "backup");
    b.mkdirs();
    return b;
  }

  public synchronized void append(Frame f) throws IOException {
    if (frames == null) {
      frames =
          Files.newBufferedWriter(
              new File(dir, "frames.jsonl").toPath(),
              StandardCharsets.UTF_8,
              StandardOpenOption.CREATE,
              StandardOpenOption.APPEND);
    }
    frames.write(LINE.writeValueAsString(f));
    frames.newLine();
    frames.flush();
  }

  public List<Frame> readFrames() throws IOException {
    File f = new File(dir, "frames.jsonl");
    List<Frame> out = new ArrayList<>();
    if (!f.exists()) {
      return out;
    }
    for (String line : Files.readAllLines(f.toPath(), StandardCharsets.UTF_8)) {
      if (!line.isBlank()) {
        out.add(MAPPER.readValue(line, Frame.class));
      }
    }
    return out;
  }

  public ObjectNode readSession() throws IOException {
    return (ObjectNode) MAPPER.readTree(new File(dir, "session.json"));
  }

  public void writeSession(JsonNode session) throws IOException {
    MAPPER.writeValue(new File(dir, "session.json"), session);
  }

  public void writeJson(File f, JsonNode n) throws IOException {
    MAPPER.writeValue(f, n);
  }

  @Override
  public synchronized void close() throws IOException {
    if (frames != null) {
      frames.close();
      frames = null;
    }
  }
}
