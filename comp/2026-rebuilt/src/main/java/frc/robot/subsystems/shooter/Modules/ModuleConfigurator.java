package frc.robot.subsystems.shooter.Modules;

import org.bobcatrobotics.Util.Tunables.Gains;
import org.bobcatrobotics.Util.Tunables.TunablePID;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ModuleConfigurator {

    private final Slot0Configs slotConfig;
    private final int motorInnerId;
    private final int motorOuterId;
    private final boolean isInnerInverted;
    private final boolean isOuterInverted;
    private final boolean isCoast;
    private final boolean isSoftLimitsEnabled;
    private final double supplyCurrentLimit;
    private final double statorCurrentLimit;
    private final double forwardSoftwareLimit;
    private final double reverseSoftwareLimit;
    private final boolean useMotionMagic;

    public double motionMagicCruiseVelocity;
    public double motionMagicAcceleration;
    public double motionMagicJerk;
    public double motionMagicExpoKV;
    public double motionMagicExpoKa;

    public ModuleConfigurator(
            Slot0Configs slotConfig,
            int motorInnerId,
            boolean isInverted,
            boolean isCoast,
            double statorCurrentLimit,
            double supplyCurrentLimit,
            boolean isSoftLimitsEnabled,
            boolean useMotionMagic) {
        // Defensive copies (REQUIRED for immutability)
        this.slotConfig = slotConfig;
        this.motorInnerId = motorInnerId;
        this.motorOuterId = motorInnerId;
        this.isInnerInverted = isInverted;
        this.isOuterInverted = isInverted;
        this.isCoast = isCoast;
        this.isSoftLimitsEnabled = isSoftLimitsEnabled;
        this.statorCurrentLimit = statorCurrentLimit;
        this.supplyCurrentLimit = supplyCurrentLimit;
        this.forwardSoftwareLimit = Double.MAX_VALUE;
        this.reverseSoftwareLimit = Double.MIN_VALUE;
        this.useMotionMagic = useMotionMagic;
    }

    public ModuleConfigurator(
            Slot0Configs slotConfig,
            int motorInnerId,
            boolean isInverted,
            boolean isCoast,
            double statorCurrentLimit,
            double supplyCurrentLimit,
            boolean isSoftLimitsEnabled,
            double forwardSoftwareLimit,
            double reverseSoftwareLimit,
            boolean useMotionMagic) {
        // Defensive copies (REQUIRED for immutability)
        this.slotConfig = slotConfig;
        this.motorInnerId = motorInnerId;
        this.motorOuterId = motorInnerId;
        this.isInnerInverted = isInverted;
        this.isOuterInverted = isInverted;
        this.isCoast = isCoast;
        this.isSoftLimitsEnabled = isSoftLimitsEnabled;
        this.statorCurrentLimit = statorCurrentLimit;
        this.supplyCurrentLimit = supplyCurrentLimit;
        this.forwardSoftwareLimit = forwardSoftwareLimit;
        this.reverseSoftwareLimit = reverseSoftwareLimit;
        this.useMotionMagic = useMotionMagic;
    }

    public ModuleConfigurator(
            Slot0Configs slotConfig,
            int motorInnerId,
            int motorOuterId,
            boolean isInnerInverted,
            boolean isOuterInverted,
            boolean isCoast,
            double statorCurrentLimit,
            double supplyCurrentLimit,
            boolean isSoftLimitsEnabled,
            boolean useMotionMagic) {
        // Defensive copies (REQUIRED for immutability)
        this.slotConfig = slotConfig;
        this.motorInnerId = motorInnerId;
        this.motorOuterId = motorOuterId;
        this.isInnerInverted = isInnerInverted;
        this.isOuterInverted = isOuterInverted;
        this.isCoast = isCoast;
        this.isSoftLimitsEnabled = isSoftLimitsEnabled;
        this.statorCurrentLimit = statorCurrentLimit;
        this.supplyCurrentLimit = supplyCurrentLimit;
        this.forwardSoftwareLimit = Double.MAX_VALUE;
        this.reverseSoftwareLimit = Double.MIN_VALUE;
        this.useMotionMagic = useMotionMagic;
    }

    /* ---------------- Getters (defensive) ---------------- */

    public Slot0Configs getSlotConfig() {
        return slotConfig;
    }

    public int getMotorOuterId() {
        return motorInnerId;
    }

    public int getMotorInnerId() {
        return motorOuterId;
    }

    public boolean isInnerInverted() {
        return isInnerInverted;
    }

    public boolean isOuterInverted() {
        return isOuterInverted;
    }

    public int getMotorId() {
        return motorInnerId;
    }

    public boolean isCoast() {
        return isCoast;
    }

    public double getSupplyCurrentLimit() {
        return statorCurrentLimit;
    }

    public double getStatorCurrentLimit() {
        return supplyCurrentLimit;
    }

    public ModuleConfigurator apply(Slot0Configs slot) {
        return new ModuleConfigurator(slot, motorInnerId, motorOuterId, isInnerInverted, isOuterInverted, isCoast,
                statorCurrentLimit,
                supplyCurrentLimit,
                isSoftLimitsEnabled, useMotionMagic);
    }

    public void applyMotionMagicConfig(double motionMagicCruiseVelocity, double motionMagicAcceleration,
            double motionMagicJerk, double motionMagicExpoKV, double motionMagicExpoKA) {
        this.motionMagicCruiseVelocity = motionMagicCruiseVelocity;
        this.motionMagicAcceleration = motionMagicAcceleration;
        this.motionMagicJerk = motionMagicJerk;
        this.motionMagicExpoKV = motionMagicExpoKV;
        this.motionMagicExpoKa = motionMagicExpoKA;
    }

    public void configureMotor(
            TalonFX motor,
            Gains pid) {

        Slot0Configs slot0 = new Slot0Configs();
        slot0 = pid.toSlot0Configs();

        TalonFXConfiguration fxConfig = new TalonFXConfiguration();
        motor.getConfigurator().apply(fxConfig); // reset

        fxConfig.Slot0 = slot0;

        fxConfig.MotorOutput.Inverted = isInnerInverted()
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        fxConfig.MotorOutput.NeutralMode = isCoast()
                ? NeutralModeValue.Coast
                : NeutralModeValue.Brake;

        fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        fxConfig.CurrentLimits.SupplyCurrentLimit = getSupplyCurrentLimit();

        fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        fxConfig.CurrentLimits.StatorCurrentLimit = getStatorCurrentLimit();

        fxConfig.TorqueCurrent.PeakForwardTorqueCurrent = getStatorCurrentLimit();
        fxConfig.TorqueCurrent.PeakReverseTorqueCurrent = -getStatorCurrentLimit();

        if (isSoftLimitsEnabled) {
            fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = isSoftLimitsEnabled;
            fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = isSoftLimitsEnabled;

            fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = forwardSoftwareLimit;
            fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = reverseSoftwareLimit;
        } else {
            fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        }

        if (useMotionMagic) {
            fxConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicCruiseVelocity; // Rotations per second
            fxConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration; // Rotations per second squared
            fxConfig.MotionMagic.MotionMagicJerk = motionMagicJerk; // Rotations per second cubed
            fxConfig.MotionMagic.MotionMagicExpo_kA = motionMagicExpoKa;
            fxConfig.MotionMagic.MotionMagicExpo_kV = motionMagicExpoKV;
        }

        motor.getConfigurator().apply(fxConfig);
    }

    public void configureMotor(
            TalonFX motor,
            TunablePID pid) {

        Slot0Configs slot0 = new Slot0Configs();
        pid.applyTo(slot0);

        TalonFXConfiguration fxConfig = new TalonFXConfiguration();
        motor.getConfigurator().apply(fxConfig); // reset

        fxConfig.Slot0 = slot0;

        fxConfig.MotorOutput.Inverted = isInnerInverted()
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        fxConfig.MotorOutput.NeutralMode = isCoast()
                ? NeutralModeValue.Coast
                : NeutralModeValue.Brake;

        fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        fxConfig.CurrentLimits.SupplyCurrentLimit = getSupplyCurrentLimit();

        fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        fxConfig.CurrentLimits.StatorCurrentLimit = getStatorCurrentLimit();
        if (isSoftLimitsEnabled) {
            fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = isSoftLimitsEnabled;
            fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = isSoftLimitsEnabled;

            fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = forwardSoftwareLimit;
            fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = reverseSoftwareLimit;
        } else {
            fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        }

        if (useMotionMagic) {
            // --- MOTION MAGIC VELOCITY CONFIGS ---
            fxConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicCruiseVelocity; // Rotations per second
            fxConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration; // Rotations per second squared
            fxConfig.MotionMagic.MotionMagicJerk = motionMagicJerk; // Rotations per second cubed
            fxConfig.MotionMagic.MotionMagicExpo_kA = motionMagicExpoKa;
            fxConfig.MotionMagic.MotionMagicExpo_kV = motionMagicExpoKV;
        }

        motor.getConfigurator().apply(fxConfig);
    }

    public void updateMotorPID(
            TalonFX motor,
            TunablePID pid) {

        Slot0Configs slot0 = new Slot0Configs();
        pid.applyTo(slot0);
        motor.getConfigurator().apply(slot0);
    }

    public void configureSignals(TalonFX motor, double freq, StatusSignal<?>... signals) {
        BaseStatusSignal.setUpdateFrequencyForAll(freq, signals);
        motor.optimizeBusUtilization();
    }
}