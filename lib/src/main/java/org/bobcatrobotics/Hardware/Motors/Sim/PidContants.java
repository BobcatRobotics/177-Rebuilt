package org.bobcatrobotics.Hardware.Motors.Sim;

import edu.wpi.first.math.util.Units;

public class PidContants {
    private double KV = 0.0;
    private double KS = 0.0;
    public PidContants(double kV_ROT,double kS){
        KV = 1.0 / Units.rotationsToRadians(1.0 / kV_ROT);
        KS = kS;
    }
    public double getKV(){
        return KV;
    }
        public double getKS(){
        return KV;
    }
}
