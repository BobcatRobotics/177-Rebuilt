package org.bobcatrobotics.Hardware.Configurators.Modules;

public class ModulePid {
    public double kP;
    public double kD;
    public double kS;
    public double kI;
    public double kV;
    public double kA;
    public ModulePid(){

    }
    public ModulePid(double kP, double kD, double kS, double kI, double kV, double kA) {
        this.kP = kP;
        this.kD = kD;
        this.kS = kS;
        this.kI = kI;
        this.kV = kV;
        this.kA = kA;
    }
    public ModulePid(double kP, double kD, double kS, double kI, double kV) {
        this.kP = kP;
        this.kD = kD;
        this.kS = kS;
        this.kI = kI;
        this.kV = kV;
    }
}
