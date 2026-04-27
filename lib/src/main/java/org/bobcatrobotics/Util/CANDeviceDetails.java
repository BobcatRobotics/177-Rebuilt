package org.bobcatrobotics.Util;


import java.util.Objects;

/**
 * Represents a CAN device with identifying details such as ID, bus name, manufacturer,
 * and subsystem. This is useful for managing devices across multiple CAN buses.
 */
public record CANDeviceDetails(
    int id,
    String bus,
    Manufacturer manufacturer,
    String subsystemName,
    DeviceType deviceType
) {
    /**
     * Enum representing the manufacturer of the CAN device.
     */
    public enum Manufacturer {
        /** Unknown vendor */
        Unknown,
        /** Thrifty vendor */
        Thrifty,
        /** Grapple vendor */
        Grapple,
        /** Pwf vendor */
        Pwf,
        /** Redux vendor */
        Redux,
        /** Rev Robotics vendor */
        Rev,
        /** CTRE (Cross The Road Electronics) vendor */
        Ctre
    }

    public enum DeviceType { NONE,TALONFX, CANCODER, PIGEON2 }

    /**
     * Constructs a {@code CANDeviceDetails} record with the specified properties.
     * All fields are required and validated to be non-null.
     *
     * @param id            the CAN ID of the device (0 to 63 typical range)
     * @param bus           the name of the CAN bus the device is on (e.g., "rio", "canivore")
     * @param manufacturer  the manufacturer of the device
     * @param subsystemName the robot subsystem this device belongs to (e.g., "drive", "arm")
     * @param deviceType the robot subsystem this device belongs to (e.g., "drive", "arm")
     * @throws NullPointerException if any of the arguments are null
     */
    public CANDeviceDetails {
        Objects.requireNonNull(id, "CAN ID cannot be null");
        Objects.requireNonNull(bus, "Bus name cannot be null");
        Objects.requireNonNull(manufacturer, "Manufacturer cannot be null");
        Objects.requireNonNull(subsystemName, "Subsystem name cannot be null");
        Objects.requireNonNull(deviceType, "Device Type cannot be null");
    }

    /**
     * Constructs a {@code CANDeviceDetails} with only an ID.
     * The bus is set to an empty string, the manufacturer to {@code Unknown},
     * and the subsystem name to an empty string.
     *
     * @param id the CAN ID of the device
     */
    public CANDeviceDetails(int id) {
        this(id, "", Manufacturer.Unknown, "",DeviceType.NONE);
    }

    /**
     * Constructs a {@code CANDeviceDetails} with an ID and bus name.
     * The manufacturer is set to {@code Unknown}, and subsystem name is empty.
     *
     * @param id  the CAN ID of the device
     * @param bus the name of the CAN bus
     */
    public CANDeviceDetails(int id, String bus) {
        this(id, bus, Manufacturer.Unknown, "",DeviceType.NONE);
    }

    /**
     * Constructs a {@code CANDeviceDetails} with ID, bus, and manufacturer.
     * The subsystem name is set to an empty string.
     *
     * @param id           the CAN ID of the device
     * @param bus          the name of the CAN bus
     * @param manufacturer the manufacturer of the device
     */
    public CANDeviceDetails(int id, String bus, Manufacturer manufacturer) {
        this(id, bus, manufacturer, "",DeviceType.NONE);
    }
    /**
     * Constructs a {@code CANDeviceDetails} with ID, bus, and manufacturer.
     * The subsystem name is set to an empty string.
     *
     * @param id           the CAN ID of the device
     * @param bus          the name of the CAN bus
     * @param manufacturer the device type
     * @param type the manufacturer of the device
     */
    public CANDeviceDetails(int id, String bus, Manufacturer manufacturer, DeviceType type) {
        this(id, bus, manufacturer, "",type);
    }
}