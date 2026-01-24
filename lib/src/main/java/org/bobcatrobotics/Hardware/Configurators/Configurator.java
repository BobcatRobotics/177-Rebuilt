package org.bobcatrobotics.Hardware.Configurators;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.databind.ObjectMapper;

import org.bobcatrobotics.Hardware.Configurators.Modules.ModuleJson;
import org.bobcatrobotics.Hardware.Configurators.Motors.MotorJson;
import org.bobcatrobotics.Hardware.Configurators.Sensors.Distance.DistanceJson;
import org.bobcatrobotics.Hardware.Configurators.Sensors.Encoders.EncoderJson;
import org.bobcatrobotics.Hardware.Configurators.Sensors.Gyros.GyroJson;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Generic configuration loader for hardware JSON files stored in the deploy directory.
 *
 * <p>
 * This class provides safe, non-crashing configuration loading with:
 * <ul>
 * <li>DriverStation error reporting</li>
 * <li>AdvantageKit logging</li>
 * <li>Automatic fallback to default configs</li>
 * </ul>
 *
 * <p>
 * Intended for use in robot initialization or enable-time reloads.
 */
public class Configurator {

    private static final ObjectMapper MAPPER = new ObjectMapper();

    private final String subsystemName;
    private final String hardwareName;
    private final String configFolderPath;
    private final String configName;

    /**
     * Creates a new hardware configurator.
     *
     * @param configFolderPath Relative path inside the deploy directory
     * @param configName JSON configuration file name
     * @param subsystemName Subsystem name for logging
     * @param hardwareName Hardware name for logging
     */
    public Configurator(String configFolderPath, String configName, String subsystemName,
            String hardwareName) {

        this.configFolderPath = configFolderPath;
        this.configName = configName;
        this.subsystemName = subsystemName;
        this.hardwareName = hardwareName;
    }

    /**
     * Loads a gyro configuration from disk.
     */
    public GyroJson loadGyroConfigurationFromFile() {
        return loadConfig(GyroJson.class, new GyroJson(), "GyroConfig");
    }

    /**
     * Loads a motor configuration from disk.
     */
    public MotorJson loadMotorConfigurationFromFile() {
        return loadConfig(MotorJson.class, new MotorJson(), "MotorConfig");
    }

    /**
     * Loads an encoder configuration from disk.
     */
    public EncoderJson loadEncoderConfigurationFromFile() {
        return loadConfig(EncoderJson.class, new EncoderJson(), "EncoderConfig");
    }

    /**
     * Loads a distance sensor configuration from disk.
     */
    public DistanceJson loadDistanceConfigurationFromFile() {
        return loadConfig(DistanceJson.class, new DistanceJson(), "DistanceConfig");
    }

    /**
     * Loads a Module configuration from disk.
     */
    public ModuleJson loadModuleConfigurationFromFile() {
        return loadConfig(ModuleJson.class, new ModuleJson(), "ModuleConfig");
    }


    /**
     * Resolves the configuration file, applying simulation-only overrides if present.
     *
     * <p>
     * If running in simulation and a {@code *.sim.json} file exists, it will be used instead of the
     * normal configuration file.
     */
    private File resolveConfigFile(File directory) {
        File realFile = new File(directory, configName);
        File simFile = new File(directory, configName.replace(".json", ".sim.json"));

        if (RobotBase.isSimulation() && simFile.exists()) {
            Logger.recordOutput(subsystemName + "/" + hardwareName + "/UsingSimOverride", true);
            return simFile;
        }

        Logger.recordOutput(subsystemName + "/" + hardwareName + "/UsingSimOverride", false);
        return realFile;
    }


    /**
     * Generic JSON configuration loader with logging and safe fallback.
     *
     * @param <T> Configuration type
     * @param clazz Class to deserialize into
     * @param defaultValue Value returned if loading fails
     * @param logPrefix AdvantageKit log prefix
     *
     * @return Loaded configuration or default value on failure
     */
    private <T> T loadConfig(Class<T> clazz, T defaultValue, String logPrefix) {

        File deployDir = Filesystem.getDeployDirectory();
        File directory = new File(deployDir, configFolderPath);
        File configFile = resolveConfigFile(directory);

        String baseLogKey = subsystemName + "/" + hardwareName + "/" + logPrefix;

        Logger.recordOutput(subsystemName + "/" + hardwareName, configFile.getAbsolutePath());

        // Directory missing
        if (!directory.exists()) {
            return fail(baseLogKey, "Config directory does not exist", directory.getAbsolutePath(),
                    defaultValue);
        }

        // File missing
        if (!configFile.exists()) {
            return fail(baseLogKey, "Config file does not exist", configFile.getAbsolutePath(),
                    defaultValue);
        }

        try {
            T config = MAPPER.readValue(configFile, clazz);
            Logger.recordOutput(baseLogKey + "/Loaded", true);
            return config;
        } catch (IOException e) {
            DriverStation.reportError(
                    "Failed to parse config file: " + configFile.getAbsolutePath(),
                    e.getStackTrace());
            Logger.recordOutput(baseLogKey + "/Loaded", false);
            Logger.recordOutput(baseLogKey + "/Error", "Parse failure");
            return defaultValue;
        }
    }

    /**
     * Handles config load failure with logging and DriverStation reporting.
     */
    private <T> T fail(String logKey, String message, String path, T defaultValue) {
        DriverStation.reportError(message + ": " + path, false);
        Logger.recordOutput(logKey + "/Loaded", false);
        Logger.recordOutput(logKey + "/Error", message);
        return defaultValue;
    }
}
