package org.bobcatrobotics.Util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;
/** 
 * Phoenix utility too attempt to apply settings a given number of times.
 */
public class PhoenixUtil {
  /** Attempts to run the command until no error is produced.
   * @param maxAttempts gets the max Attermpts to try to apply configuration
   * @param command the comnmand you are looking too attempt
   */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) {
        break;
      }
    }
  }
}
