package frc.robot.util;

import com.ctre.phoenix6.configs.Slot0Configs;

/**
 * Immutable container for PID + Feedforward gains.
 *
 * Includes:
 *  - kP, kI, kD (PID)
 *  - kS, kV, kA (Feedforward)
 */
public final class Gains {

    private final double kP;
    private final double kI;
    private final double kD;
    private final double kS;
    private final double kV;
    private final double kA;

    private Gains(Builder builder) {
        this.kP = builder.kP;
        this.kI = builder.kI;
        this.kD = builder.kD;
        this.kS = builder.kS;
        this.kV = builder.kV;
        this.kA = builder.kA;
    }

    // Getters
    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }
    public double getKS() { return kS; }
    public double getKV() { return kV; }
    public double getKA() { return kA; }

        /**
     * Converts this Gains object into a CTRE Slot0Configs object.
     */
    public Slot0Configs toSlot0Configs() {
        return new Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKV(kV)
                .withKA(kA);
    }

    /**
     * Populates an existing Slot0Configs instance.
     */
    public void applyTo(Slot0Configs slot0) {
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;
        slot0.kS = kS;
        slot0.kV = kV;
        slot0.kA = kA;
    }
    

    // Builder Pattern for clean construction
    public static class Builder {
        private double kP;
        private double kI;
        private double kD;
        private double kS;
        private double kV;
        private double kA;

        public Builder kP(double kP) {
            this.kP = kP;
            return this;
        }

        public Builder kI(double kI) {
            this.kI = kI;
            return this;
        }

        public Builder kD(double kD) {
            this.kD = kD;
            return this;
        }

        public Builder kS(double kS) {
            this.kS = kS;
            return this;
        }

        public Builder kV(double kV) {
            this.kV = kV;
            return this;
        }

        public Builder kA(double kA) {
            this.kA = kA;
            return this;
        }

        public Gains build() {
            return new Gains(this);
        }
    }

    @Override
    public String toString() {
        return String.format(
            "Gains[kP=%.4f, kI=%.4f, kD=%.4f, kS=%.4f, kV=%.4f, kA=%.4f]",
            kP, kI, kD, kS, kV, kA
        );
    }
}
