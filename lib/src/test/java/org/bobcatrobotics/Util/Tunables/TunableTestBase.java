package org.bobcatrobotics.Util.Tunables;

import edu.wpi.first.networktables.NetworkTableInstance;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

public abstract class TunableTestBase {

  protected NetworkTableInstance nt;

  @BeforeEach
  void setupNT() {
    nt = NetworkTableInstance.create();
    nt.startLocal();
  }

  @AfterEach
  void shutdownNT() {
    nt.stopLocal();
  }
}

