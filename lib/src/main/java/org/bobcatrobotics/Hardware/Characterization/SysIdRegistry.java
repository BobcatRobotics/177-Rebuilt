package org.bobcatrobotics.Hardware.Characterization;

import java.util.HashMap;
import java.util.Map;

public class SysIdRegistry {

    private final Map<String, SysIdModule> modules = new HashMap<>();

    public void register(String name, SysIdModule module) {
        modules.put(name, module);
    }

    public SysIdModule get(String name) {
        return modules.get(name);
    }

    public Map<String, SysIdModule> getAll() {
        return modules;
    }
}