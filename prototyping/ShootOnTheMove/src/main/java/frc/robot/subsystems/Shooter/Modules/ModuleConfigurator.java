package frc.robot.subsystems.Shooter.Modules;

import com.ctre.phoenix6.configs.Slot0Configs;

public final class ModuleConfigurator {
    private final Slot0Configs slotConfig;
    private final int motorLeftId;
    private final int motorRightId;
    private final boolean isInverted;
    private final boolean isCoast;
    private final double currentLimit;
    public ModuleConfigurator(
            Slot0Configs slotConfig,
            int motorId,
            boolean isInverted,
            boolean isCoast,
            double currentLimit) {
        // Defensive copies (REQUIRED for immutability)
        this.slotConfig = slotConfig;
        this.motorLeftId = motorId;
        this.motorRightId = motorId;
        this.isInverted = isInverted;
        this.isCoast = isCoast;
        this.currentLimit = currentLimit;
    }
    public ModuleConfigurator(
            Slot0Configs slotConfig,
            int motorLeftId,
            int motorRightId,
            boolean isInverted,
            boolean isCoast,
            double currentLimit) {
        // Defensive copies (REQUIRED for immutability)
        this.slotConfig = slotConfig;
        this.motorLeftId = motorLeftId;
        this.motorRightId = motorRightId;
        this.isInverted = isInverted;
        this.isCoast = isCoast;
        this.currentLimit = currentLimit;
    }

    /* ---------------- Getters (defensive) ---------------- */

    public Slot0Configs getSlotConfig() {
        return slotConfig;
    }

    public int getMotorRightId() {
        return motorRightId;
    }
    public int getMotorLeftId() {
        return motorLeftId;
    }

    public boolean isInverted() {
        return isInverted;
    }

    public boolean isCoast() {
        return isCoast;
    }

    public double getCurrentLimit() {
        return currentLimit;
    }

    public ModuleConfigurator apply(Slot0Configs slot){
        return new ModuleConfigurator(slot, motorLeftId, isInverted, isCoast, currentLimit) ;
    }

}
