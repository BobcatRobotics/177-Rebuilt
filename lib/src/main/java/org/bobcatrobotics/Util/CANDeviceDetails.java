package org.bobcatrobotics.Util;


import java.util.Objects;

/**
 * Represents a CAN device with identifying details such as ID, bus name, manufacturer,
 * and subsystem. This is useful for managing devices across multiple CAN buses.
 */
public record CANDeviceDetails(
    int id,
    String bus,
    Hardware hardware,
    Manufacturer manufacturer,
    String subsystemName,
    boolean status
) {

        /**
     * Enum representing the manufacturer of the CAN device.
     */
    public enum Hardware {        
        /** Unknown vendor */
        Unknown,
        /** TalonFX vendor */
        TalonFX,
        /** CANcoder vendor */
        CANcoder,
        /** Pigeon2 vendor */
        Pigeon2
    }
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

    /**
     * Constructs a {@code CANDeviceDetails} record with the specified properties.
     * All fields are required and validated to be non-null.
     *
     * @param id            the CAN ID of the device (0 to 63 typical range)
     * @param bus           the name of the CAN bus the device is on (e.g., "rio", "canivore")
     * @param hardware  the hardware of the device
     * @param manufacturer  the manufacturer of the device
     * @param subsystemName the robot subsystem this device belongs to (e.g., "drive", "arm")
     * @param status the status of the device on the bus (IE connected / disconnected)
     * @throws NullPointerException if any of the arguments are null
     */
    public CANDeviceDetails {
        Objects.requireNonNull(id, "CAN ID cannot be null");
        Objects.requireNonNull(bus, "Bus name cannot be null");
        Objects.requireNonNull(hardware, "Hardware cannot be null");
        Objects.requireNonNull(manufacturer, "Manufacturer cannot be null");
        Objects.requireNonNull(subsystemName, "Subsystem name cannot be null");
    }

    /**
     * Constructs a {@code CANDeviceDetails} with only an ID.
     * The bus is set to an empty string, the manufacturer to {@code Unknown},
     * and the subsystem name to an empty string.
     *
     * @param id the CAN ID of the device
     */
    public CANDeviceDetails(int id) {
        this(id, "", Hardware.Unknown, Manufacturer.Unknown, "",false);
    }

    /**
     * Constructs a {@code CANDeviceDetails} with an ID and bus name.
     * The manufacturer is set to {@code Unknown}, and subsystem name is empty.
     *
     * @param id  the CAN ID of the device
     * @param bus the name of the CAN bus
     */
    public CANDeviceDetails(int id, String bus) {
        this(id, bus, Hardware.Unknown,Manufacturer.Unknown, "",false);
    }

    /**
     * Constructs a {@code CANDeviceDetails} with ID, bus, and manufacturer.
     * The subsystem name is set to an empty string.
     *
     * @param id           the CAN ID of the device
     * @param bus          the name of the CAN bus
     * @param hardware     the hardware type
     * @param manufacturer the manufacturer of the device
     */
    public CANDeviceDetails(int id, String bus, Hardware hardware, Manufacturer manufacturer) {
        this(id, bus, hardware, manufacturer, "",false);
    }

        /**
     * Constructs a {@code CANDeviceDetails} with ID, bus, and manufacturer.
     * The subsystem name is set to an empty string.
     *
     * @param id           the CAN ID of the device
     * @param bus          the name of the CAN bus
     * @param hardware     the hardware type
     * @param manufacturer the manufacturer of the device
     * @param status the status of the device
     */
    public CANDeviceDetails(int id, String bus, Hardware hardware, Manufacturer manufacturer, boolean status) {
        this(id, bus, hardware, manufacturer, "",status);
    }
}