package org.bobcatrobotics.Hardware.CAN;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;

public class CANLogger {
    private final List<CANStatusReader> readers;

    private static final double errorTimeThreshold = 0.5;

    private final Timer initialTimer = new Timer();
    private final Timer errorTimer = new Timer();

    private final Alert errorAlert = new Alert(
        "CAN errors detected, robot may not be controllable.",
        AlertType.kError
    );

    public CANLogger(List<CANStatusReader> readers) {
        this.readers = readers;

        initialTimer.restart();
        errorTimer.restart();
    }

    public void periodic() {
        boolean anyError = false;

        for (var reader : readers) {
            var statusOpt = reader.getStatus();

            if (statusOpt.isPresent()) {
                var s = statusOpt.get();

                Logger.recordOutput("CAN/" + s.name + "/Utilization", s.utilization);
                Logger.recordOutput("CAN/" + s.name + "/BusOffCount", s.busOffCount);
                Logger.recordOutput("CAN/" + s.name + "/TxFullCount", s.txFullCount);
                Logger.recordOutput("CAN/" + s.name + "/ReceiveErrorCount", s.receiveErrorCount);
                Logger.recordOutput("CAN/" + s.name + "/TransmitErrorCount", s.transmitErrorCount);

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
    }
}