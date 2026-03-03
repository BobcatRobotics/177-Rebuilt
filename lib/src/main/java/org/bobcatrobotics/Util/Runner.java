package org.bobcatrobotics.Util;

import edu.wpi.first.wpilibj.Notifier;

public class Runner {
  private Runnable callback;
  private Notifier executer = null;
  private double periodSeconds = 0.00;
  /**
   * Constructs an instance of the runner, this will allow you to take an method and execute
   * periodically given a configuration you pass in.
   *
   * @param callback
   * @param periodSeconds
   */
  public Runner(Runnable callback, double periodSeconds) {
    this.callback = callback;
    this.periodSeconds = periodSeconds;
  }
  /** Initializes and starts the task running at a configurable period of time */
  public void init() {
    executer = new Notifier(callback);
    executer.startPeriodic(periodSeconds);
  }
}
