package org.bobcatrobotics.Hardware.CAN;

import java.util.Optional;

public class RioReaderAdapter implements CANStatusReader {
    private final RioReader reader = new RioReader();

    @Override
    public Optional<CANStatusData> getStatus() {
        var statusOpt = reader.getStatus();
        if (statusOpt.isEmpty()) return Optional.empty();

        var s = statusOpt.get();

        CANStatusData data = new CANStatusData();
        data.name = "RIO";
        data.utilization = s.percentBusUtilization;
        data.busOffCount = 0;
        data.txFullCount = 0;
        data.receiveErrorCount = s.receiveErrorCount;
        data.transmitErrorCount = s.transmitErrorCount;
        data.isOk = (s.receiveErrorCount == 0 && s.transmitErrorCount == 0);

        return Optional.of(data);
    }
}