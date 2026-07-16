package org.bobcatrobotics.Util.Tunables;

/**
 * Generic interface for a tunable value.
 *
 * @param <T> Value type
 */
public interface Tunable<T> {

  /** Returns the current value */
  T get();

  /** Returns true if the value has changed since last read */
  boolean hasChanged();

  /** Returns the NetworkTables key */
  String getKey();

  /** 
   * Reset this tunable back to its default value 
   * 
   * Example :
   *  Reset all tunables on enable ( optional )
   *  DefaultTunableRegistry.getInstance().resetAllToDefaults();
   *  Reset all tunables on debug command ( optional )  
   *  new InstantCommand(
    () -> DefaultTunableRegistry.getInstance().resetAllToDefaults());
   */
  void resetToDefault();
}
