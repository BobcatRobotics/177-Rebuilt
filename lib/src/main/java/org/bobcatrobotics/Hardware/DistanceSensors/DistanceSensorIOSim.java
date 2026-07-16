package org.bobcatrobotics.Hardware.DistanceSensors;

import org.bobcatrobotics.Util.CANDeviceDetails;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Simulation implementation of {@link DistanceSensorIO}.
 * <p>
 * This class uses WPILib {@link SimDevice} entries to emulate a distance
 * sensor in simulation. It exposes simulated distance and validity values
 * that can be driven by tests or simulation code while maintaining the same
 * IO contract used by real hardware implementations.
 * </p>
 */
public class DistanceSensorIOSim implements DistanceSensorIO {

    /** Alert displayed when the simulated sensor reports an invalid state. */
    private final Alert sensorUnpluggedAlert;

    /** CAN device metadata associated with this sensor. */
    private final CANDeviceDetails details;

    /** WPILib simulation device backing this sensor. */
    private final SimDevice simDevice;

    /** Simulated distance measurement in meters. */
    private final SimDouble simDistanceMeters;

    /** Simulated validity flag. */
    private final SimBoolean simValid;

    /** Last successfully read distance value. */
    private double lastDistanceM = 0.0;

    /** Cached validity state of the sensor. */
    private boolean valid = false;

    /**
     * Creates a new simulated distance sensor IO.
     *
     * @param details CAN device metadata used to uniquely identify the sensor
     */
    public DistanceSensorIOSim(CANDeviceDetails details) {
        this.details = details;

        sensorUnpluggedAlert =
                new Alert(
                        details.subsystemName() + " TOF Range Joystick unplugged!",
                        AlertType.kWarning
                );

        simDevice = SimDevice.create("DistanceSensor", details.id());

        if (simDevice != null) {
            simDistanceMeters =
                    simDevice.createDouble(
                            "DistanceMeters",
                            SimDevice.Direction.kInput,
                            0.0
                    );

            simValid =
                    simDevice.createBoolean(
                            "Valid",
                            SimDevice.Direction.kInput,
                            true
                    );
        } else {
            // Simulation disabled (e.g. running on real hardware)
            simDistanceMeters = null;
            simValid = null;
        }
    }

    /**
     * Updates the provided input structure with the latest simulated values.
     *
     * @param inputs the input data structure to populate
     */
    @Override
    public void updateInputs(DistanceSensorIOInputs inputs) {
        if (simDevice == null) {
            inputs.isValid = false;
            inputs.distanceInMeters = simDistanceMeters.get();
            inputs.connected = false;
            return;
        }

        inputs.isValid = simValid.get();
        inputs.distanceInMeters = getDistanceMeters();
        inputs.connected = true;

        sensorUnpluggedAlert.set(!inputs.isValid);
    }

    /**
     * Returns the current simulated distance measurement.
     * <p>
     * If the simulated value cannot be read, the last known distance
     * is returned and the sensor is marked invalid.
     * </p>
     *
     * @return the simulated distance in meters
     */
    @Override
    public double getDistanceMeters() {
        try {
            valid = true;
            lastDistanceM = simDistanceMeters.get();
        } catch (Exception ex) {
            DriverStation.reportWarning(
                    "PWF TOF read failed: " + ex,
                    false
            );
            valid = false;
        }

        return lastDistanceM;
    }

    /**
     * Returns whether the sensor is currently reporting valid data.
     *
     * @return {@code true} if the sensor is valid, otherwise {@code false}
     */
    @Override
    public boolean isValid() {
        return valid;
    }

    /**
     * Sets the simulated distance value.
     *
     * @param meters the distance to report, in meters
     */
    @Override
    public void simSetDistance(double meters) {
        if (simDistanceMeters != null) {
            simDistanceMeters.set(meters);
        }
    }

    /**
     * Sets whether the simulated sensor reports valid data.
     *
     * @param valid {@code true} to mark the sensor as valid, {@code false} otherwise
     */
    public void setValid(boolean valid) {
        if (simValid != null) {
            simValid.set(valid);
        }
    }
}
