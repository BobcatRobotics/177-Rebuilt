package frc.vision.cal;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;

/**
 * Minimal client for the Limelight REST API (port 5807). See
 * https://docs.limelightvision.io/docs/docs-limelight/apis/rest-http-api
 *
 * <p>Only {@link #updatePipeline} and {@link #uploadFieldmap} change anything on the camera, and
 * the tool only calls them after the user has confirmed (or, for sweeps, without flush so nothing
 * is saved).
 */
public final class LimelightHttp {
  private static final ObjectMapper MAPPER = new ObjectMapper();
  private static final HttpClient CLIENT =
      HttpClient.newBuilder().connectTimeout(Duration.ofMillis(800)).build();

  public final String name;
  private final String base;

  public LimelightHttp(String name, String host) {
    this.name = name;
    this.base = "http://" + host + ":5807";
  }

  public JsonNode results() throws IOException {
    return getJson("/results");
  }

  public JsonNode status() throws IOException {
    return getJson("/status");
  }

  public JsonNode hwreport() throws IOException {
    return getJson("/hwreport");
  }

  public JsonNode pipelineAtIndex(int index) throws IOException {
    return getJson("/pipeline-atindex?index=" + index);
  }

  /** Applies settings to the running pipeline. {@code flush=true} saves them to the camera. */
  public void updatePipeline(JsonNode settings, boolean flush) throws IOException {
    post("/update-pipeline" + (flush ? "?flush=1" : ""), MAPPER.writeValueAsString(settings));
  }

  public void uploadFieldmap(String fmapJson) throws IOException {
    post("/upload-fieldmap", fmapJson);
  }

  public JsonNode getJson(String path) throws IOException {
    HttpRequest req =
        HttpRequest.newBuilder(URI.create(base + path)).timeout(Duration.ofSeconds(2)).GET().build();
    return MAPPER.readTree(send(req));
  }

  private void post(String path, String body) throws IOException {
    HttpRequest req =
        HttpRequest.newBuilder(URI.create(base + path))
            .timeout(Duration.ofSeconds(3))
            .header("Content-Type", "application/json")
            .POST(HttpRequest.BodyPublishers.ofString(body))
            .build();
    send(req);
  }

  private String send(HttpRequest req) throws IOException {
    try {
      HttpResponse<String> resp = CLIENT.send(req, HttpResponse.BodyHandlers.ofString());
      if (resp.statusCode() / 100 != 2) {
        throw new IOException(name + " " + req.uri().getPath() + " -> HTTP " + resp.statusCode());
      }
      return resp.body();
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IOException("interrupted", e);
    } catch (IOException e) {
      if (e.getMessage() == null) {
        throw new IOException(e.getClass().getSimpleName() + " talking to " + req.uri().getHost(), e);
      }
      throw e;
    }
  }
}
