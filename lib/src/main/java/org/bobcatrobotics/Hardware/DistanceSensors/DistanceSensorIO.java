package org.bobcatrobotics.Hardware.DistanceSensors;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for distance sensors.
 * <p>
 * Implementations of this interface provide a common API for both real and
 * simulated distance sensors. Data is passed to higher-level subsystems using
 * the {@link DistanceSensorIOInputs} structure, which is automatically logged
 * by AdvantageKit.
 * </p>
 */
public interface DistanceSensorIO {

    /**
     * Container for distance sensor input data.
     * <p>
     * This class is annotated with {@link AutoLog} to enable automatic
     * logging via AdvantageKit.
     * </p>
     */
    @AutoLog
    public static class DistanceSensorIOInputs {

        /** Measured distance in meters. */
        public double distanceInMeters = 0.00;

        /** Whether the sensor reading is valid. */
        public boolean isValid = false;

        /** Whether the sensor is currently connected. */
        public boolean connected = false;
    }

    /**
     * Updates the provided input structure with the latest sensor values.
     * <p>
     * Implementations should populate all fields of {@link DistanceSensorIOInputs}
     * on every call.
     * </p>
     *
     * @param inputs the input data structure to update
     */
    public default void updateInputs(DistanceSensorIOInputs inputs) {}

    /**
     * Returns the most recent distance measurement.
     *
     * @return distance in meters
     */
    public default double getDistanceMeters() {
        return 0.00;
    }

    /**
     * Returns whether the sensor is currently reporting valid data.
     *
     * @return {@code true} if the sensor is valid, otherwise {@code false}
     */
    public default boolean isValid() {
        return false;
    }

    /**
     * Returns the current status of the sensor.
     * <p>
     * This may represent connection health, internal fault status,
     * or another implementation-defined condition.
     * </p>
     *
     * @return {@code true} if the sensor status is healthy
     */
    public default boolean getStatus() {
        return false;
    }

    /**
     * Sets the simulated distance value.
     * <p>
     * This method is intended for simulation-only implementations.
     * Real hardware implementations may safely ignore this call.
     * </p>
     *
     * @param distanceInMeters distance to report, in meters
     */
    public default void simSetDistance(double distanceInMeters) {}
}
