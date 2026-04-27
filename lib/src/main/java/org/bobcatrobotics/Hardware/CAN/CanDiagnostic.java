package org.bobcatrobotics.Hardware.CAN;

import java.util.ArrayList;
import java.util.List;

import org.bobcatrobotics.Util.CANDeviceDetails;
import org.bobcatrobotics.Util.CANDeviceDetails.DeviceType;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

public class CanDiagnostic {

    enum HardwareType { TALONFX, CANCODER, PIGEON2, NONE }

    static class Device {
        String name;
        int id;
        String bus;
        HardwareType type;
        Object device;

        int failureCount = 0;
        int successCount = 0;
        long lastSeen = 0;

        Device(String name, int id, String bus, HardwareType type) {
            this.name = name;
            this.id = id;
            this.bus = bus;
            this.type = type;

            switch (type) {
                case TALONFX -> device = new TalonFX(id, bus);
                case CANCODER -> device = new CANcoder(id, bus);
                case PIGEON2 -> device = new Pigeon2(id, bus);
            }
        }

        StatusCode check() {
            StatusCode status = StatusCode.GeneralError;

            if (device instanceof TalonFX t) status = t.getVersion().getStatus();
            if (device instanceof CANcoder c) status = c.getVersion().getStatus();
            if (device instanceof Pigeon2 p) status = p.getVersion().getStatus();

            if (status.isOK()) {
                successCount++;
                lastSeen = System.currentTimeMillis();
            } else {
                failureCount++;
            }

            return status;
        }

        double health() {
            int total = successCount + failureCount;
            if (total == 0) return 1.0;
            return (double) successCount / total;
        }

        String key() {
            // Clean key for logging (no spaces)
            return name.replaceAll("\\s+", "");
        }
    }

    private final List<Device> devices = new ArrayList<>();

    public CanDiagnostic(List<CANDeviceDetails> detailedDevices){
        for (CANDeviceDetails device : detailedDevices) {
            HardwareType devType = HardwareType.NONE;
            switch(device.deviceType()){
                case CANCODER:
                    devType = HardwareType.CANCODER;
                    break;
                case PIGEON2:
                    devType = HardwareType.PIGEON2;
                    break;
                case TALONFX:
                     devType = HardwareType.TALONFX;
                    break;
                default:
                    devType = HardwareType.NONE;
                    break;
            }
            add(device.bus() + "_" + device.id(), device.id() , device.bus(), devType );
        }
    }


    private void add(String name, int id, String bus, HardwareType type) {
        devices.add(new Device(name, id, bus, type));
    }

    public void periodic() {
        long now = System.currentTimeMillis();
        List<Integer> failedIndices = new ArrayList<>();

        for (int i = 0; i < devices.size(); i++) {
            Device d = devices.get(i);
            StatusCode status = d.check();

            boolean connected = status.isOK();
            double health = d.health();
            double lastSeenSec = (now - d.lastSeen) / 1000.0;

            String base = "CAN/" + d.key();

            // ✅ Named channels
            Logger.recordOutput(base + "/Connected", connected);
            Logger.recordOutput(base + "/Health", health);
            Logger.recordOutput(base + "/LastSeenSec", lastSeenSec);
            Logger.recordOutput(base + "/Bus", d.bus);
            Logger.recordOutput(base + "/ID", d.id);

            if (!connected) {
                failedIndices.add(i);
            }
        }

        inferBreak(failedIndices);
        detectIntermittent(now);
    }

    private void inferBreak(List<Integer> failed) {
        if (failed.isEmpty()) {
            Logger.recordOutput("CAN/Status", "OK");
            Logger.recordOutput("CAN/BreakConfidence", 0.0);
            return;
        }

        int start = failed.get(0);

        if (start == 0) {
            Logger.recordOutput("CAN/Break", "StartOfBus");
            Logger.recordOutput("CAN/BreakConfidence", 0.6);
            return;
        }

        Device before = devices.get(start - 1);
        Device after = devices.get(start);

        String msg = before.name + " -> " + after.name;

        // Confidence increases if multiple consecutive failures
        int clusterSize = 1;
        for (int i = 1; i < failed.size(); i++) {
            if (failed.get(i) == failed.get(i - 1) + 1) {
                clusterSize++;
            }
        }

        double confidence = Math.min(1.0, 0.5 + 0.1 * clusterSize);

        Logger.recordOutput("CAN/Break", msg);
        Logger.recordOutput("CAN/BreakConfidence", confidence);
    }

    private void detectIntermittent(long now) {
        for (Device d : devices) {
            if (d.lastSeen == 0) continue;

            boolean intermittent = (now - d.lastSeen) > 500;

            Logger.recordOutput(
                "CAN/" + d.key() + "/Intermittent",
                intermittent
            );
        }
    }
}