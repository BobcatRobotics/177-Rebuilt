package org.bobcatrobotics.Hardware.DistanceSensors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem wrapper for a distance sensor.
 * <p>
 * This class owns a {@link DistanceSensorIO} implementation and is responsible
 * for periodically updating inputs and forwarding them to AdvantageKit for
 * logging. Higher-level code should interact with this class rather than
 * accessing hardware or simulation implementations directly.
 * </p>
 */
public class DistanceSensor extends SubsystemBase {

    /** Hardware abstraction layer for the distance sensor. */
    private final DistanceSensorIO io;

    /** Automatically logged sensor inputs. */
    private final DistanceSensorIOInputsAutoLogged inputs =
            new DistanceSensorIOInputsAutoLogged();

    /**
     * Creates a new distance sensor subsystem.
     *
     * @param io the IO implementation backing this sensor (real or simulated)
     */
    public DistanceSensor(DistanceSensorIO io) {
        this.io = io;
    }

    /**
     * Periodic update method.
     * <p>
     * Reads the latest sensor values from the IO layer and forwards them
     * to AdvantageKit for logging.
     * </p>
     */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Distance Sensor", inputs);
    }

    /**
     * Returns the current distance measurement.
     *
     * @return distance in meters
     */
    public double getDistance() {
        return io.getDistanceMeters();
    }

    /**
     * Returns the current sensor status.
     *
     * @return {@code true} if the sensor status is healthy
     */
    public boolean getStatus() {
        return io.getStatus();
    }

    /**
     * Sets the simulated distance value.
     * <p>
     * This method is intended for simulation use only and is ignored
     * by real hardware implementations.
     * </p>
     *
     * @param distanceInMeters distance to report, in meters
     */
    public void simSetDistance(double distanceInMeters) {
        io.simSetDistance(distanceInMeters);
    }
}
