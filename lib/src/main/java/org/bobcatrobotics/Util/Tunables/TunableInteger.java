package org.bobcatrobotics.Util.Tunables;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * TunableDouble allows live tuning of numeric values through NetworkTables
 * (AdvantageScope, Shuffleboard, Glass) with minimal loop overhead.
 */
public final class TunableInteger {

    /** Toggle this to false for competition */
    private static boolean tuningEnabled = true;

    private final String key;
    private final int defaultValue;

    private final NetworkTableEntry entry;

    // Cached values to prevent unnecessary NT access
    private int lastValue;
    private int cachedValue;

    /**
     * Create a tunable double under the "/Tuning" table.
     *
     * @param name          NetworkTables key name
     * @param defaultValue default value if NT is not present
     */
    public TunableInteger(String name, int defaultValue) {
        this("Shuffleboard", name, defaultValue);
    }

    /**
     * Create a tunable double under a custom table.
     *
     * @param tableName     NetworkTables table name
     * @param name          key name
     * @param defaultValue default value
     */
    public TunableInteger(String tableName, String name, int defaultValue) {
        this.key = tableName + "/" + name;
        this.defaultValue = defaultValue;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(tableName);
        this.entry = table.getEntry(name);

        // Initialize NT only once
        entry.setDefaultInteger(defaultValue);

        this.cachedValue = defaultValue;
        this.lastValue = defaultValue;
    }

    /**
     * Get the current value.
     * Safe to call every loop.
     */
    public int get() {
        if (!tuningEnabled) {
            return defaultValue;
        }

        cachedValue = entry.getNumber(defaultValue).intValue();
        return cachedValue;
    }

    /**
     * Returns true if the value changed since the last check.
     * Useful for updating PID controllers only when needed.
     */
    public boolean hasChanged() {
        int current = get();
        if (current != lastValue) {
            lastValue = current;
            return true;
        }
        return false;
    }

    /**
     * Get the value and update the cached change state.
     * Slightly faster than calling get() + hasChanged().
     */
    public double getAndUpdate() {
        int current = get();
        lastValue = current;
        return current;
    }

    /**
     * Reset the value back to its default.
     */
    public void reset() {
        entry.setInteger(defaultValue);
        cachedValue = defaultValue;
        lastValue = defaultValue;
    }

    /**
     * Enable or disable tuning globally.
     */
    public static void setTuningEnabled(boolean enabled) {
        tuningEnabled = enabled;
    }
}
