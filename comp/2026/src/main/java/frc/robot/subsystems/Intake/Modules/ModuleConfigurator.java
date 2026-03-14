package frc.robot.subsystems.Intake.Modules;


import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.bobcatrobotics.Util.Tunables.Gains;
// import org.bobcatrobotics.Util.Tunables.TunablePID;

public final class ModuleConfigurator {
    private final Slot0Configs slotConfig;
    private final int motorInnerId;
    private final int motorOuterId;
    private final boolean isInnerInverted;
    private final boolean isOuterInverted;
    private final boolean isCoast;
    private final double currentLimit;
    private final double peakForward;
    private final double peakReverse;

    public ModuleConfigurator(
            Slot0Configs slotConfig,
            int motorInnerId,
            boolean isInverted,
            boolean isCoast,
            double currentLimit,
            double peakForward,
            double peakReverse) {
        // Defensive copies (REQUIRED for immutability)
        this.slotConfig = slotConfig;
        this.motorInnerId = motorInnerId;
        this.motorOuterId = motorInnerId;
        this.isInnerInverted = isInverted;
        this.isOuterInverted = isInverted;
        this.isCoast = isCoast;
        this.currentLimit = currentLimit;
        this.peakForward = peakForward;
        this.peakReverse = peakReverse;
    }

    public ModuleConfigurator(
            Slot0Configs slotConfig,
            int motorInnerId,
            int motorOuterId,
            boolean isInnerInverted,
            boolean isOuterInverted,
            boolean isCoast,
            double currentLimit,
            double peakForward,
            double peakReverse) {
        // Defensive copies (REQUIRED for immutability)
        this.slotConfig = slotConfig;
        this.motorInnerId = motorInnerId;
        this.motorOuterId = motorOuterId;
        this.isInnerInverted = isInnerInverted;
        this.isOuterInverted = isOuterInverted;
        this.isCoast = isCoast;
        this.currentLimit = currentLimit;
        this.peakForward = peakForward;
        this.peakReverse = peakReverse;
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
                currentLimit, peakForward, peakReverse);
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


        fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        fxConfig.CurrentLimits.StatorCurrentLimit = getCurrentLimit();
        fxConfig.TorqueCurrent.PeakForwardTorqueCurrent = peakForward;
        fxConfig.TorqueCurrent.PeakReverseTorqueCurrent = peakReverse;
        fxConfig.CurrentLimits.SupplyCurrentLimit = 60;
        fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(fxConfig);
    }  


    public void configureSignals(TalonFX motor,double freq, StatusSignal<?>... signals) {
        BaseStatusSignal.setUpdateFrequencyForAll(freq, signals);
        motor.optimizeBusUtilization();
    }

}
