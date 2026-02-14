package frc.robot.commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter.Shooter;

public class shooterCharacterizationCommands {
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>
     * This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization_Flywheel(Shooter shooter) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient
                Commands.run(() -> {
                    shooter.runCharacterization_Flywheel(0.0);
                }, shooter).withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(() -> {
                    double voltage = timer.get() * FF_RAMP_RATE;
                    shooter.runCharacterization_Flywheel(voltage);
                    velocitySamples.add(shooter.getFFCharacterizationVelocity_Flywheel());
                    voltageSamples.add(voltage);
                }, shooter)

                        // When cancelled, calculate and print results
                        .finallyDo(() -> {
                            int n = velocitySamples.size();
                            double sumX = 0.0;
                            double sumY = 0.0;
                            double sumXY = 0.0;
                            double sumX2 = 0.0;
                            for (int i = 0; i < n; i++) {
                                sumX += velocitySamples.get(i);
                                sumY += voltageSamples.get(i);
                                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                            }
                            double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                            double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
                            double kP = 0.2 * (1/kV);

                            NumberFormat formatter = new DecimalFormat("#0.00000");
                            System.out.println("********** Flywheel FF Characterization Results **********");
                            System.out.println("\tkS: " + formatter.format(kS));
                            System.out.println("\tkV: " + formatter.format(kV));
                            Logger.recordOutput("Shooter/Characterization/Flywheel/kS",formatter.format(kS));
                            Logger.recordOutput("Shooter/Characterization/Flywheel/kV",formatter.format(kV));
                            Logger.recordOutput("Shooter/Characterization/Flywheel/kP",formatter.format(kP));
                        }));
    }


    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>
     * This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization_Backspin(Shooter shooter) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient
                Commands.run(() -> {
                    shooter.runCharacterization_Backspin(0.0);
                }, shooter).withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(() -> {
                    double voltage = timer.get() * FF_RAMP_RATE;
                    shooter.runCharacterization_Backspin(voltage);
                    velocitySamples.add(shooter.getFFCharacterizationVelocity_Backspin());
                    voltageSamples.add(voltage);
                }, shooter)

                        // When cancelled, calculate and print results
                        .finallyDo(() -> {
                            int n = velocitySamples.size();
                            double sumX = 0.0;
                            double sumY = 0.0;
                            double sumXY = 0.0;
                            double sumX2 = 0.0;
                            for (int i = 0; i < n; i++) {
                                sumX += velocitySamples.get(i);
                                sumY += voltageSamples.get(i);
                                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                            }
                            double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                            double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
                            double kP = 0.2 * (1/kV);

                            NumberFormat formatter = new DecimalFormat("#0.00000");
                            System.out.println("********** Backspin FF Characterization Results **********");
                            System.out.println("\tkS: " + formatter.format(kS));
                            System.out.println("\tkV: " + formatter.format(kV));
                            Logger.recordOutput("Shooter/Characterization/Backspin/kS",formatter.format(kS));
                            Logger.recordOutput("Shooter/Characterization/Backspin/kV",formatter.format(kV));
                            Logger.recordOutput("Shooter/Characterization/Backspin/kP",formatter.format(kP));
                        }));
    }


    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>
     * This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization_Intake(Shooter shooter) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient
                Commands.run(() -> {
                    shooter.runCharacterization_Intake(0.0);
                }, shooter).withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(() -> {
                    double voltage = timer.get() * FF_RAMP_RATE;
                    shooter.runCharacterization_Intake(voltage);
                    velocitySamples.add(shooter.getFFCharacterizationVelocity_Intake());
                    voltageSamples.add(voltage);
                }, shooter)

                        // When cancelled, calculate and print results
                        .finallyDo(() -> {
                            int n = velocitySamples.size();
                            double sumX = 0.0;
                            double sumY = 0.0;
                            double sumXY = 0.0;
                            double sumX2 = 0.0;
                            for (int i = 0; i < n; i++) {
                                sumX += velocitySamples.get(i);
                                sumY += voltageSamples.get(i);
                                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                            }
                            double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                            double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
                            double kP = 0.2 * (1/kV);

                            NumberFormat formatter = new DecimalFormat("#0.00000");
                            System.out.println("********** Intake FF Characterization Results **********");
                            System.out.println("\tkS: " + formatter.format(kS));
                            System.out.println("\tkV: " + formatter.format(kV));
                            Logger.recordOutput("Shooter/Characterization/Intake/kS",formatter.format(kS));
                            Logger.recordOutput("Shooter/Characterization/Intake/kV",formatter.format(kV));
                            Logger.recordOutput("Shooter/Characterization/Intake/kP",formatter.format(kP));
                        }));
    }

}
