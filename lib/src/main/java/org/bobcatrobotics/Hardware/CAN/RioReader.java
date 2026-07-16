package org.bobcatrobotics.Hardware.CAN;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.hal.can.CANStatus;
import java.util.Optional;

public class RioReader {
  private final Thread thread;
  private Optional<CANStatus> status = Optional.empty();

  public RioReader() {
    thread =
        new Thread(
            () -> {
              while (true) {
                var statusTemp = Optional.of(RobotController.getCANStatus());

                synchronized (this) {
                  status = statusTemp;
                }

                try {
                  Thread.sleep(400);
                } catch (InterruptedException e) {
                  e.printStackTrace();
                }
              }
            });

    thread.setName("RioReader");
    thread.setDaemon(true);
    thread.start();
  }

  public synchronized Optional<CANStatus> getStatus() {
    return status;
  }
}