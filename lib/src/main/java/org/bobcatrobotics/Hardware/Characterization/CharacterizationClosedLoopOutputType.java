package org.bobcatrobotics.Hardware.Characterization;

import java.util.HashMap;

/**
 * Supported closed-loop output types.
 */
public enum CharacterizationClosedLoopOutputType {
    Voltage(0),
    /** Requires Pro */
    TorqueCurrentFOC(1);

    public final int value;

    CharacterizationClosedLoopOutputType(int initValue) {
        this.value = initValue;
    }

    private static HashMap<Integer, CharacterizationClosedLoopOutputType> _map = null;
    static {
        _map = new HashMap<Integer, CharacterizationClosedLoopOutputType>();
        for (CharacterizationClosedLoopOutputType type : CharacterizationClosedLoopOutputType.values()) {
            _map.put(type.value, type);
        }
    }

    /**
     * Gets CharacterizationClosedLoopOutputType from specified value
     *
     * @param value Value of CharacterizationClosedLoopOutputType
     * @return CharacterizationClosedLoopOutputType of specified value
     */
    public static CharacterizationClosedLoopOutputType valueOf(int value) {
        CharacterizationClosedLoopOutputType retval = _map.get(value);
        if (retval != null){
            return retval;
            }
        return CharacterizationClosedLoopOutputType.values()[0];
    }
}
