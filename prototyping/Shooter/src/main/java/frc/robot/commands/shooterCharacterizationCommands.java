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

    private static double[] solve3x3(double[][] m, double[] b) {

        double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

        if (Math.abs(det) < 1e-9) {
            throw new RuntimeException("Characterization failed: singular matrix (bad data)");
        }

        double invdet = 1.0 / det;

        double[][] inv = new double[3][3];

        inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invdet;
        inv[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * invdet;
        inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;

        inv[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * invdet;
        inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
        inv[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * invdet;

        inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invdet;
        inv[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * invdet;
        inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invdet;

        return new double[] {
                inv[0][0] * b[0] + inv[0][1] * b[1] + inv[0][2] * b[2],
                inv[1][0] * b[0] + inv[1][1] * b[1] + inv[1][2] * b[2],
                inv[2][0] * b[0] + inv[2][1] * b[1] + inv[2][2] * b[2]
        };
    }

    private static Command characterize(
            Shooter shooter,
            Runnable zeroVoltage,
            java.util.function.DoubleConsumer applyVoltage,
            java.util.function.DoubleSupplier velocitySupplier,
            String logPath,
            String name) {

        List<Double> velocitySamples = new LinkedList<>();
        List<Double> accelerationSamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();

        Timer timer = new Timer();
        final double dt = 0.02;
        final double[] lastVelocity = { 0.0 };

        return Commands.sequence(

                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    accelerationSamples.clear();
                    voltageSamples.clear();
                    lastVelocity[0] = 0.0;
                }),

                Commands.run(zeroVoltage::run, shooter)
                        .withTimeout(FF_START_DELAY),

                Commands.runOnce(timer::restart),

                Commands.run(() -> {

                    double voltage = timer.get() * FF_RAMP_RATE;
                    applyVoltage.accept(voltage);

                    double velocity = velocitySupplier.getAsDouble();
                    double acceleration = (velocity - lastVelocity[0]) / dt;
                    lastVelocity[0] = velocity;

                    // 🔥 Ignore very low speeds (reduces kS noise)
                    if (Math.abs(velocity) > 5.0) {
                        velocitySamples.add(velocity);
                        accelerationSamples.add(acceleration);
                        voltageSamples.add(voltage);
                    }

                }, shooter)

                        .finallyDo(() -> {

                            int n = velocitySamples.size();
                            if (n < 15) {
                                System.out.println("Not enough valid samples!");
                                return;
                            }

                            double sum1 = 0, sumW = 0, sumA = 0;
                            double sumWW = 0, sumAA = 0, sumWA = 0;
                            double sumV = 0, sumVW = 0, sumVA = 0;

                            for (int i = 0; i < n; i++) {
                                double w = velocitySamples.get(i);
                                double a = accelerationSamples.get(i);
                                double v = voltageSamples.get(i);

                                sum1 += 1;
                                sumW += w;
                                sumA += a;
                                sumWW += w * w;
                                sumAA += a * a;
                                sumWA += w * a;
                                sumV += v;
                                sumVW += v * w;
                                sumVA += v * a;
                            }

                            double[][] m = {
                                    { sum1, sumW, sumA },
                                    { sumW, sumWW, sumWA },
                                    { sumA, sumWA, sumAA }
                            };

                            double[] b = { sumV, sumVW, sumVA };

                            double[] gains = solve3x3(m, b);

                            double kS = gains[0];
                            double kV = gains[1];
                            double kA = gains[2];

                            double kP = 0.2 * (1.0 / kV);
                            double kD = kA * kP;

                            NumberFormat formatter = new DecimalFormat("#0.00000");

                            System.out.println("********** " + name + " Characterization **********");
                            System.out.println("\tkS: " + formatter.format(kS));
                            System.out.println("\tkV: " + formatter.format(kV));
                            System.out.println("\tkA: " + formatter.format(kA));
                            System.out.println("\tkP: " + formatter.format(kP));
                            System.out.println("\tkD: " + formatter.format(kD));

                            // ✅ Log raw doubles
                            Logger.recordOutput(logPath + "/kS", kS);
                            Logger.recordOutput(logPath + "/kV", kV);
                            Logger.recordOutput(logPath + "/kA", kA);
                            Logger.recordOutput(logPath + "/kP", kP);
                            Logger.recordOutput(logPath + "/kD", kD);
                        }));
    }

    public static Command feedforwardCharacterization_Flywheel(Shooter shooter) {
        return characterize(
                shooter,
                () -> shooter.runCharacterization_Flywheel(0.0),
                shooter::runCharacterization_Flywheel,
                shooter::getFFCharacterizationVelocity_Flywheel,
                "Shooter/Characterization/Flywheel",
                "Flywheel");
    }

    public static Command feedforwardCharacterization_Backspin(Shooter shooter) {
        return characterize(
                shooter,
                () -> shooter.runCharacterization_Backspin(0.0),
                shooter::runCharacterization_Backspin,
                shooter::getFFCharacterizationVelocity_Backspin,
                "Shooter/Characterization/Backspin",
                "Backspin");
    }

    public static Command feedforwardCharacterization_Intake(Shooter shooter) {
        return characterize(
                shooter,
                () -> shooter.runCharacterization_Intake(0.0),
                shooter::runCharacterization_Intake,
                shooter::getFFCharacterizationVelocity_Intake,
                "Shooter/Characterization/Intake",
                "Intake");
    }

    public static Command characterizeAll(Shooter shooter) {
        return feedforwardCharacterization_Flywheel(shooter).alongWith(feedforwardCharacterization_Flywheel(shooter))
                .alongWith(feedforwardCharacterization_Flywheel(shooter))
                .alongWith(feedforwardCharacterization_Flywheel(shooter));
    }
}
