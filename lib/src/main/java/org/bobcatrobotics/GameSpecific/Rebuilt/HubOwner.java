package org.bobcatrobotics.GameSpecific.Rebuilt;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Which alliance currently owns / has an active hub.
 */
public enum HubOwner {
  RED,
  BLUE,
  NONE;

  public boolean isOurAlliance(Alliance alliance) {
    return (this == RED && alliance == Alliance.Red)
        || (this == BLUE && alliance == Alliance.Blue);
  }
}
