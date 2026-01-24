package org.bobcatrobotics.Hardware.Configurators.Modules;

public class ModuleJson {
    public ModuleConfigurationJson module = new ModuleConfigurationJson();
    public ModulePid steerGains = new ModulePid();
    public ModulePid driveGains = new ModulePid();
    public String name = "";
}
