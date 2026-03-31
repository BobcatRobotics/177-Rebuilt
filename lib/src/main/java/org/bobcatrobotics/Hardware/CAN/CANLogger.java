package org.bobcatrobotics.Hardware.CAN;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class CANLogger {
    public CanivoreReader canivoreReader;
    private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
    private static final double canivoreErrorTimeThreshold = 0.5;
    private final Timer canInitialErrorTimer = new Timer();
    private final Timer canErrorTimer = new Timer();
    private final Timer canivoreErrorTimer = new Timer();

    private final Alert canErrorAlert = new Alert("CAN errors detected, robot may not be controllable.",
            AlertType.kError);
    private final Alert canivoreErrorAlert = new Alert("CANivore errors detected, robot may not be controllable.",
            AlertType.kError);

    public CANLogger(CANBus canvivoreName) {
        canivoreReader = new CanivoreReader(canvivoreName.getName());
        // Reset alert timers
        canInitialErrorTimer.restart();
        canErrorTimer.restart();
        canivoreErrorTimer.restart();
    }

    public void periodic() {
        // Check CAN status
        var canStatus = RobotController.getCANStatus();
        if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
            canErrorTimer.restart();
        }
        canErrorAlert.set(
                !canErrorTimer.hasElapsed(canErrorTimeThreshold)
                        && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));

        // Log CANivore status
            var canivoreStatus = canivoreReader.getStatus();
            if (canivoreStatus.isPresent()) {
                Logger.recordOutput("CANivoreStatus/Status", canivoreStatus.get().Status.getName());
                Logger.recordOutput("CANivoreStatus/Utilization", canivoreStatus.get().BusUtilization);
                Logger.recordOutput("CANivoreStatus/OffCount", canivoreStatus.get().BusOffCount);
                Logger.recordOutput("CANivoreStatus/TxFullCount", canivoreStatus.get().TxFullCount);
                Logger.recordOutput("CANivoreStatus/ReceiveErrorCount", canivoreStatus.get().REC);
                Logger.recordOutput("CANivoreStatus/TransmitErrorCount", canivoreStatus.get().TEC);
                if (!canivoreStatus.get().Status.isOK()
                        || canStatus.transmitErrorCount > 0
                        || canStatus.receiveErrorCount > 0) {
                    canivoreErrorTimer.restart();
                }
            }
            canivoreErrorAlert.set(
                    !canivoreErrorTimer.hasElapsed(canivoreErrorTimeThreshold)
                            && !canInitialErrorTimer.hasElapsed(canErrorTimeThreshold));
    }
}
