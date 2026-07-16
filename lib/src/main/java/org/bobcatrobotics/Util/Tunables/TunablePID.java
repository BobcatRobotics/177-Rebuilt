package org.bobcatrobotics.Util.Tunables;

import com.ctre.phoenix6.configs.Slot0Configs;

public class TunablePID {
  final TunableDouble kP, kI, kD, kS, kV, kA;

  public TunablePID(String path, Gains defaults) {
    kP = new TunableDouble(path + "/kP", defaults.getKP());
    kI = new TunableDouble(path + "/kI", defaults.getKI());
    kD = new TunableDouble(path + "/kD", defaults.getKD());
    kS = new TunableDouble(path + "/kS", defaults.getKS());
    kV = new TunableDouble(path + "/kV", defaults.getKV());
    kA = new TunableDouble(path + "/kA", defaults.getKA());
  }

  public void applyTo(Slot0Configs slot) {
    slot.kP = kP.get();
    slot.kI = kI.get();
    slot.kD = kD.get();
    slot.kS = kS.get();
    slot.kV = kV.get();
    slot.kA = kA.get();
  }

  public boolean hasChanged() {
    return kP.hasChanged()
        || kI.hasChanged()
        || kD.hasChanged()
        || kS.hasChanged()
        || kV.hasChanged()
        || kA.hasChanged();
  }
}


