package org.bobcatrobotics.Hardware.CAN;

import java.util.Optional;

public class CanivoreReaderAdapter implements CANStatusReader {
    private final CanivoreReader reader;

    public CanivoreReaderAdapter(String busName) {
        this.reader = new CanivoreReader(busName);
    }

    @Override
    public Optional<CANStatusData> getStatus() {
        var statusOpt = reader.getStatus();
        if (statusOpt.isEmpty()) return Optional.empty();

        var s = statusOpt.get();

        CANStatusData data = new CANStatusData();
        data.name = s.Status.getName();
        data.utilization = s.BusUtilization;
        data.busOffCount = s.BusOffCount;
        data.txFullCount = s.TxFullCount;
        data.receiveErrorCount = s.REC;
        data.transmitErrorCount = s.TEC;
        data.isOk = s.Status.isOK();

        return Optional.of(data);
    }
}