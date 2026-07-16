package org.bobcatrobotics.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.CANBus;

public class CANBusStatusLogger {
    private final CANDeviceDetails details;
    private final CANBus bus;

    public CANBusStatusLogger(CANDeviceDetails details) {
        this.details = details;
        bus = new CANBus(details.bus());
    }

    public void logStatus() {
        var status = bus.getStatus();
        Logger.recordOutput("CANBusStatus/" + bus.getName(), status.Status);
        SmartDashboard.putString("CANBusStatus/" + bus.getName(), status.Status.getName());
    }
}