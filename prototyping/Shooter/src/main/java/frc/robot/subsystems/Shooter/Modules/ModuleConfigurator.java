package frc.robot.subsystems.Shooter.Modules;

import com.ctre.phoenix6.configs.Slot0Configs;

public final class ModuleConfigurator {
    private final Slot0Configs slotConfig;
    private final int motorInnerId;
    private final int motorOuterId;
    private final boolean isInnerInverted;
    private final boolean isOuterInverted;
    private final boolean isCoast;
    private final double currentLimit;

    public ModuleConfigurator(
            Slot0Configs slotConfig,
            int motorInnerId,
            boolean isInverted,
            boolean isCoast,
            double currentLimit) {
        // Defensive copies (REQUIRED for immutability)
        this.slotConfig = slotConfig;
        this.motorInnerId = motorInnerId;
        this.motorOuterId = motorInnerId;
        this.isInnerInverted = isInverted;
        this.isOuterInverted = isInverted;
        this.isCoast = isCoast;
        this.currentLimit = currentLimit;
    }

    public ModuleConfigurator(
            Slot0Configs slotConfig,
            int motorInnerId,
            int motorOuterId,
            boolean isInnerInverted,
            boolean isOuterInverted,
            boolean isCoast,
            double currentLimit) {
        // Defensive copies (REQUIRED for immutability)
        this.slotConfig = slotConfig;
        this.motorInnerId = motorInnerId;
        this.motorOuterId = motorOuterId;
        this.isInnerInverted = isInnerInverted;
        this.isOuterInverted = isOuterInverted;
        this.isCoast = isCoast;
        this.currentLimit = currentLimit;
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

    public double getCurrentLimit() {
        return currentLimit;
    }

    public ModuleConfigurator apply(Slot0Configs slot) {
        return new ModuleConfigurator(slot, motorInnerId, motorOuterId, isInnerInverted, isOuterInverted, isCoast,
                currentLimit);
    }

}
