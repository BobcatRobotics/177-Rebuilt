package org.bobcatrobotics.Util.Tunables;

import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Default global registry for all Tunables.
 */
public final class DefaultTunableRegistry implements TunableRegistry {

    private static final DefaultTunableRegistry INSTANCE = new DefaultTunableRegistry();

    private final Map<String, Tunable<?>> tunables = new LinkedHashMap<>();

    private DefaultTunableRegistry() {
    }

    public static DefaultTunableRegistry getInstance() {
        return INSTANCE;
    }

    @Override
    public synchronized void register(Tunable<?> tunable) {
        if (tunables.containsKey(tunable.getKey())) {
            throw new IllegalStateException(
                    "Duplicate Tunable key: " + tunable.getKey());
        }
        tunables.put(tunable.getKey(), tunable);
    }

    @Override
    public synchronized Collection<Tunable<?>> getAll() {
        return Collections.unmodifiableCollection(tunables.values());
    }

    /**
     * Reset this tunable back to its default value
     * 
     * Example :
     * Reset all tunables on enable ( optional )
     * DefaultTunableRegistry.getInstance().resetAllToDefaults();
     * Reset all tunables on debug command ( optional )
     * new InstantCommand(
     * () -> DefaultTunableRegistry.getInstance().resetAllToDefaults());
     */
    @Override
    public synchronized void resetAllToDefaults() {
        tunables.values().forEach(Tunable::resetToDefault);
    }
}
