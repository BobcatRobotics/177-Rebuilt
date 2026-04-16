package org.bobcatrobotics.Hardware.CAN;

import java.util.Optional;

public interface CANStatusReader {
    Optional<CANStatusData> getStatus();
}