package org.bobcatrobotics.Subsystems.Swerve;

import org.bobcatrobotics.Hardware.Configurators.Configurator;
import org.bobcatrobotics.Hardware.Configurators.Modules.ModuleJson;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public class ModuleWrapper {
    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private final double kCoupleRatio;
    private final double kDriveGearRatio;
    private final double kSteerGearRatio;
    private final Slot0Configs steerGains;
    private final Slot0Configs driveGains;
    private final boolean steerMotorInverted;
    private final boolean driveMotorInverted;
    private final boolean encoderInverted;

    public ModuleWrapper(String filename, String name) {
        Configurator configurator = new Configurator("Swerve", filename, "Swerve", name);
        ModuleJson config = configurator.loadModuleConfigurationFromFile();
        this.kCoupleRatio = config.module.kCoupleRatio;
        this.kDriveGearRatio = config.module.kDriveGearRatio;
        this.kSteerGearRatio = config.module.kSteerGearRatio;
        // The steer motor uses any SwerveModule.SteerRequestType control request with
        // the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        steerGains = new Slot0Configs()
                .withKP(config.steerGains.kP).withKI(config.steerGains.kI).withKD(config.steerGains.kD)
                .withKS(config.steerGains.kS).withKV(config.steerGains.kV).withKA(config.steerGains.kA)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        driveGains = new Slot0Configs()
                .withKP(config.driveGains.kP).withKI(config.driveGains.kI).withKD(config.driveGains.kD)
                .withKS(config.driveGains.kS).withKV(config.driveGains.kV);
        this.steerMotorInverted = config.module.steerInverted ;
        this.driveMotorInverted = config.module.driveInverted ;
        this.encoderInverted = config.module.encoderInverted ;

    }

    public ModuleWrapper(double kCoupleRatio, double kDriveGearRatio, double kSteerGearRatio, Slot0Configs steerGains,
            Slot0Configs driveGains, boolean steerInverted, boolean driveInverted, boolean encoderInverted) {
        this.kCoupleRatio = kCoupleRatio;
        this.kDriveGearRatio = kDriveGearRatio;
        this.kSteerGearRatio = kSteerGearRatio;
        this.steerGains = steerGains;
        this.driveGains = driveGains;

        this.steerMotorInverted = steerInverted;
        this.driveMotorInverted = driveInverted ;
        this.encoderInverted = encoderInverted;

    }

    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> addModuleConstants(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator) {
        return ConstantCreator
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withDriveMotorInverted(driveMotorInverted)
                .withSteerMotorInverted(steerMotorInverted)
                .withEncoderInverted(encoderInverted);
    }
}
