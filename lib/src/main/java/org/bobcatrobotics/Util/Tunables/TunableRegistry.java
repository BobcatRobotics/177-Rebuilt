package org.bobcatrobotics.Util.Tunables;

import java.util.Collection;

public interface TunableRegistry {

  void register(Tunable<?> tunable);

  /** Returns all registered tunables */
  Collection<Tunable<?>> getAll();

 /** Reset all registered tunables to their default values */
  void resetAllToDefaults();
}