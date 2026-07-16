package org.bobcatrobotics.GameSpecific.Rebuilt;

/**
 * Immutable snapshot of hub state.
 */
public final class HubData {
  public final HubOwner owner;
  public final double timeRemaining;

  public HubData(HubOwner owner, double timeRemaining) {
    this.owner = owner;
    this.timeRemaining = timeRemaining;
  }

  /** Convenience */
  public boolean isActive() {
    return owner != HubOwner.NONE;
  }
}
