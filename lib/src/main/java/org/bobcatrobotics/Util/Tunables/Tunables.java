package org.bobcatrobotics.Util.Tunables;


/**
 * Factory for creating tunable values.
 */
public final class Tunables {

  private Tunables() {}

  public static TunableDouble doubleValue(String path,
            double defaultValue
            ) {
    return new TunableDouble(path, defaultValue);
  }
}
