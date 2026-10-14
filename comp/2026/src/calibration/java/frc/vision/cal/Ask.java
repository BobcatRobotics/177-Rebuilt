package frc.vision.cal;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;

/** All terminal input goes through one reader so prompts and spot marking don't steal lines. */
final class Ask {
  private static final BufferedReader IN =
      new BufferedReader(new InputStreamReader(System.in, StandardCharsets.UTF_8));

  private Ask() {}

  /** True if a full or partial line is waiting (used while also watching the dashboard button). */
  static boolean ready() throws IOException {
    return IN.ready();
  }

  static String line() throws IOException {
    String l = IN.readLine();
    return l == null ? "" : l.trim();
  }

  /** Asks a y/n question. Returns "y", "n" or "skip" (empty answer / anything else). */
  static String yesNo(String question) throws IOException {
    System.out.print(question + " [y/n, Enter = skip]: ");
    System.out.flush();
    String a = line().toLowerCase();
    if (a.startsWith("y")) {
      return "y";
    }
    if (a.startsWith("n")) {
      return "n";
    }
    return "skip";
  }

  /** Requires the word "yes" for anything that changes a camera. */
  static boolean confirm(String question) throws IOException {
    System.out.print(question + " Type yes to continue: ");
    System.out.flush();
    return line().equalsIgnoreCase("yes");
  }
}
