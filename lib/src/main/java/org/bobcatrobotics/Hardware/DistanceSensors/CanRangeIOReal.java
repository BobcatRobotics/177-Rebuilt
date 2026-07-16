package org.bobcatrobotics.Hardware.DistanceSensors;

import org.bobcatrobotics.Util.CANDeviceDetails;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Real implementation of a CANrange distance sensor using CTRE hardware.
 * 
 * <p>
 * This class communicates with a physical CANrange sensor over CAN bus and provides distance
 * readings in meters. It also manages a warning alert if the sensor is disconnected.
 * </p>
 */
public class CanRangeIOReal implements DistanceSensorIO {
    /** Alert displayed if the sensor is unplugged or disconnected */
    private final Alert sensorUnpluggedAlert;

    /** CAN device details containing ID, bus, and subsystem information */
    private final CANDeviceDetails details;

    /** The underlying CANrange sensor object */
    private final CANrange sensor;

    /** Last measured distance in meters */
    private double lastDistanceM = 0.0;

    /** Whether the last reading was valid */
    private boolean valid = false;

    /**
     * Constructs a CanRangeIOReal instance.
     * 
     * @param details the CAN device details for this sensor, including ID and bus
     */
    public CanRangeIOReal(CANDeviceDetails details) {
        this.details = details;
        CANBus bus = new CANBus(details.bus());
        sensor = new CANrange(details.id(), bus);
        sensorUnpluggedAlert =
                new Alert(details.subsystemName() + " Can Range" + " Joystick unplugged!",
                        AlertType.kWarning);
    }

    /**
     * Updates the inputs structure with the latest sensor values.
     * 
     * @param inputs the {@link DistanceSensorIOInputs} object to populate with current readings
     */
    public void updateInputs(DistanceSensorIOInputs inputs) {
        inputs.distanceInMeters = getDistanceMeters();
        inputs.isValid = isValid();
        inputs.connected = sensor.isConnected();
        sensorUnpluggedAlert.set(!inputs.connected);
    }

    /**
     * Gets the most recent distance measured by the sensor in meters.
     * 
     * <p>
     * If the sensor reading fails, the previous valid distance is returned and
     * {@link #isValid()} will return false.
     * </p>
     * 
     * @return the distance in meters
     */
    public double getDistanceMeters() {
        try {
            valid = true;
            lastDistanceM = sensor.getDistance().getValueAsDouble();
        } catch (Exception ex) {
            valid = false;
        }
        return lastDistanceM;
    }

    /**
     * Checks if the last sensor reading was valid.
     * 
     * @return true if the last reading was successful, false if an error occurred
     */
    public boolean isValid() {
        return valid;
    }

    /**
     * Returns the connection status of the CANrange sensor.
     * 
     * @return true if the sensor is currently connected, false otherwise
     */
    public boolean getStatus() {
        return sensor.isConnected();
    }

    /**
     * Sets the distance value in simulation.
     * 
     * <p>
     * This method does nothing for the real sensor and is provided for interface compatibility
     * with simulated implementations.
     * </p>
     * 
     * @param distanceInMeters the distance to set in simulation
     */
    public void simSetDistance(double distanceInMeters) {
    }
}
