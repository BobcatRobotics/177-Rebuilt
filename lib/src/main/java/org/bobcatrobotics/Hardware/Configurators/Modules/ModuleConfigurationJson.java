package org.bobcatrobotics.Hardware.Configurators.Modules;

public class ModuleConfigurationJson {
    public ModuleConfigurationJson() {
    }

    public ModuleConfigurationJson(String type, double kCoupleRatio, double kDriveGearRatio, double kSteerGearRatio) {
    }

    /** The device type, e.g. pigeon/pigeon2/sparkmax/talonfx/navx */
    public String type = "";
    public double kCoupleRatio = 0.00;
    public double kDriveGearRatio = 0.00;
    public double kSteerGearRatio = 0.00;
    public boolean driveInverted = false;
    public boolean steerInverted = false;
    public boolean encoderInverted = false;
}
