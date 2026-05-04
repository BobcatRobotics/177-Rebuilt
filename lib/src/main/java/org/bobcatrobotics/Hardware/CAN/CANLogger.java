package org.bobcatrobotics.Hardware.CAN;

import org.bobcatrobotics.Util.CANDeviceDetails;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class CANLogger {
    private final Map<String, CANStatusReader> readers;
    private  Map<String, CanDiagnostic> CanDiagnostic = new HashMap<>();
    private static final double errorTimeThreshold = 0.5;

    private final Timer initialTimer = new Timer();
    private final Timer errorTimer = new Timer();

    private final Alert errorAlert = new Alert(
        "CAN errors detected, robot may not be controllable.",
        AlertType.kError
    );

    public CANLogger(List<CANStatusReader> canReaders) {
        this.readers = canReaders.stream().collect(
            Collectors.toMap(CANStatusReader::getName,reader->reader)
        );

        initialTimer.restart();
        errorTimer.restart();

        
    }

    public void setupCanDiagnosticLogger(String name, List<CANDeviceDetails> devices){

        CanDiagnostic.put(name, new CanDiagnostic(devices));
    }

    public void sortByIdOrder(String name, List<Integer> ids){
        CanDiagnostic diagnostic = CanDiagnostic.get(name);
        diagnostic.sortByIds(ids);
    }

    public void periodic() {
        boolean anyError = false;

        for (Map.Entry<String, CANStatusReader> entry : readers.entrySet()) {
            String name = entry.getKey();
            CANStatusReader reader = entry.getValue();

            // use name + reader
            var statusOpt = reader.getStatus();

            if (statusOpt.isPresent()) {
                var s = statusOpt.get();

                Logger.recordOutput("CAN/" + name + "/Utilization", s.utilization);
                Logger.recordOutput("CAN/" + name + "/BusOffCount", s.busOffCount);
                Logger.recordOutput("CAN/" + name + "/TxFullCount", s.txFullCount);
                Logger.recordOutput("CAN/" + name + "/ReceiveErrorCount", s.receiveErrorCount);
                Logger.recordOutput("CAN/" + name + "/TransmitErrorCount", s.transmitErrorCount);

                if (!s.isOk) {
                    anyError = true;
                }
            }
        }

        if (anyError) {
            errorTimer.restart();
        }

        errorAlert.set(
            !errorTimer.hasElapsed(errorTimeThreshold)
            && !initialTimer.hasElapsed(errorTimeThreshold)
        );

        CanDiagnostic.get("rio").periodic();
        CanDiagnostic.get("CANivore").periodic();
    }
}