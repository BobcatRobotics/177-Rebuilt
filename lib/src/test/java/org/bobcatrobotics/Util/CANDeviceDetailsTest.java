package org.bobcatrobotics.Util;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class CANDeviceDetailsTest {

    @Test
    void testFullConstructor() {
        CANDeviceDetails device = new CANDeviceDetails(
            5,
            "rio",
            CANDeviceDetails.Manufacturer.Rev,
            "drive"
        );

        assertEquals(5, device.id());
        assertEquals("rio", device.bus());
        assertEquals(CANDeviceDetails.Manufacturer.Rev, device.manufacturer());
        assertEquals("drive", device.subsystemName());
    }

    @Test
    void testConstructorWithIdOnly() {
        CANDeviceDetails device = new CANDeviceDetails(10);

        assertEquals(10, device.id());
        assertEquals("", device.bus());
        assertEquals(CANDeviceDetails.Manufacturer.Unknown, device.manufacturer());
        assertEquals("", device.subsystemName());
    }

    @Test
    void testConstructorWithIdAndBus() {
        CANDeviceDetails device = new CANDeviceDetails(15, "canivore");

        assertEquals(15, device.id());
        assertEquals("canivore", device.bus());
        assertEquals(CANDeviceDetails.Manufacturer.Unknown, device.manufacturer());
        assertEquals("", device.subsystemName());
    }

    @Test
    void testConstructorWithIdBusAndManufacturer() {
        CANDeviceDetails device = new CANDeviceDetails(20, "rio", CANDeviceDetails.Manufacturer.Ctre);

        assertEquals(20, device.id());
        assertEquals("rio", device.bus());
        assertEquals(CANDeviceDetails.Manufacturer.Ctre, device.manufacturer());
        assertEquals("", device.subsystemName());
    }

    @Test
    void testNullBusThrowsException() {
        NullPointerException exception = assertThrows(
            NullPointerException.class,
            () -> new CANDeviceDetails(1, null, CANDeviceDetails.Manufacturer.Rev, "drive")
        );
        assertEquals("Bus name cannot be null", exception.getMessage());
    }

    @Test
    void testNullManufacturerThrowsException() {
        NullPointerException exception = assertThrows(
            NullPointerException.class,
            () -> new CANDeviceDetails(1, "rio", null, "drive")
        );
        assertEquals("Manufacturer cannot be null", exception.getMessage());
    }

    @Test
    void testNullSubsystemNameThrowsException() {
        NullPointerException exception = assertThrows(
            NullPointerException.class,
            () -> new CANDeviceDetails(1, "rio", CANDeviceDetails.Manufacturer.Rev, null)
        );
        assertEquals("Subsystem name cannot be null", exception.getMessage());
    }
}
