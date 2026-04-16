package org.bobcatrobotics.Hardware.CAN;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import java.util.Optional;

public class CanivoreReader {
  private final CANBus canBus;
  private final Thread thread;
  private Optional<CANBusStatus> status = Optional.empty();

  public CanivoreReader(String canBusName) {
    canBus = new CANBus(canBusName);

    thread =
        new Thread(
            () -> {
              while (true) {
                var rawStatus = canBus.getStatus();
                var statusTemp = Optional.ofNullable(rawStatus);

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

    thread.setName("CanivoreReader");
    thread.setDaemon(true); // important
    thread.start();
  }

  public synchronized Optional<CANBusStatus> getStatus() {
    return status;
  }
}