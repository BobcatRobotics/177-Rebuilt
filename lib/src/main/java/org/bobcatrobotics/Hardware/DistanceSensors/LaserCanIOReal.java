package org.bobcatrobotics.Hardware.DistanceSensors;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;

import org.bobcatrobotics.Util.CANDeviceDetails;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Real hardware implementation of {@link DistanceSensorIO} for the LaserCAN sensor.
 * <p>
 * This class interfaces with the Grapple Robotics LaserCAN over the CAN bus
 * and converts the reported distance from millimeters to meters using
 * WPILib's units API.
 * </p>
 */
public class LaserCanIOReal implements DistanceSensorIO {

    /** Alert displayed when the LaserCAN reports an invalid or disconnected state. */
    private final Alert sensorUnpluggedAlert;

    /** CAN device metadata associated with this sensor. */
    private final CANDeviceDetails details;

    /** LaserCAN hardware instance. */
    private final LaserCan sensor;

    /** Last successfully read distance value in meters. */
    private double lastDistanceM = 0.0;

    /** Cached validity state of the sensor. */
    private boolean valid = false;

    /**
     * Creates a new LaserCAN distance sensor IO.
     *
     * @param details CAN device metadata used to identify the sensor
     */
    public LaserCanIOReal(CANDeviceDetails details) {
        this.details = details;

        sensor = new LaserCan(details.id());

        sensorUnpluggedAlert =
                new Alert(
                        details.subsystemName() + " Laser Can Joystick unplugged!",
                        AlertType.kWarning
                );
    }

    /**
     * Updates the provided input structure with the latest sensor values.
     *
     * @param inputs the input data structure to populate
     */
    @Override
    public void updateInputs(DistanceSensorIOInputs inputs) {
        inputs.distanceInMeters = getDistanceMeters();
        inputs.isValid = isValid();
        inputs.connected = inputs.isValid;

        sensorUnpluggedAlert.set(!inputs.isValid);
    }

    /**
     * Returns the current distance measurement from the LaserCAN.
     * <p>
     * The value is read in millimeters from the hardware and converted
     * to meters using the WPILib units framework.
     * </p>
     *
     * @return distance in meters
     */
    @Override
    public double getDistanceMeters() {
        try {
            valid = true;
            lastDistanceM =
                    Millimeters
                            .of(sensor.getMeasurement().distance_mm)
                            .in(Meters);
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
     * Returns the current health status of the sensor.
     *
     * @return {@code true} if the sensor status is healthy
     */
    @Override
    public boolean getStatus() {
        return isValid();
    }

    /**
     * Simulation hook for setting distance values.
     * <p>
     * This method is intentionally left empty for real hardware
     * implementations.
     * </p>
     *
     * @param distanceInMeters ignored
     */
    @Override
    public void simSetDistance(double distanceInMeters) {}
}
