package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake.Intake;

import org.ejml.simple.SimpleMatrix;

public class intakeCharacterizationCommands {
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    private static double[] solve3x3(double[][] m, double[] b) {
        SimpleMatrix A = new SimpleMatrix(m);
        SimpleMatrix B = new SimpleMatrix(3, 1, true, b);

        SimpleMatrix X = A.solve(B);

        return new double[] {
                X.get(0),
                X.get(1),
                X.get(2)
        };
    }

    private static double[] solve4x4(double[][] m, double[] b) {

        if (m.length != 4 || m[0].length != 4 || b.length != 4) {
            throw new IllegalArgumentException("Expected 4x4 matrix and length-4 vector.");
        }

        SimpleMatrix A = new SimpleMatrix(m);
        SimpleMatrix B = new SimpleMatrix(4, 1, true, b);

        SimpleMatrix X = A.solve(B);

        return new double[] {
                X.get(0),
                X.get(1),
                X.get(2),
                X.get(3)
        };
    }

    /**
     * is based on the formula V=kS+kV⋅ω+kA⋅α
     * kD≈kA⋅kP
     * @param intake
     * @param zeroVoltage
     * @param applyVoltage
     * @param velocitySupplier
     * @param logPath
     * @param name
     * @return
     */
    private static Command characterize(
            Intake intake,
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

                Commands.run(zeroVoltage::run, intake)
                        .withTimeout(FF_START_DELAY),

                Commands.runOnce(timer::restart),

                Commands.run(() -> {

                    double voltage = timer.get() * FF_RAMP_RATE;
                    applyVoltage.accept(voltage);

                    double velocity = velocitySupplier.getAsDouble();
                    double acceleration = (velocity - lastVelocity[0]) / dt;
                    lastVelocity[0] = velocity;

                    // Ignore very low speeds (reduces kS noise)
                    if (Math.abs(velocity) > 5.0) {
                        velocitySamples.add(velocity);
                        accelerationSamples.add(acceleration);
                        voltageSamples.add(voltage);
                    }

                }, intake)

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

                            // Log raw doubles
                            Logger.recordOutput(logPath + "/kS", kS);
                            Logger.recordOutput(logPath + "/kV", kV);
                            Logger.recordOutput(logPath + "/kA", kA);
                            Logger.recordOutput(logPath + "/kP", kP);
                            Logger.recordOutput(logPath + "/kD", kD);
                        }));
    }

/**
 * Is based on the formula V=kS+kVω+kAα+kGcos(θ)
 * kD≈kA⋅kP 
 * @param intake
 * @param zeroVoltage
 * @param applyVoltage
 * @param angleRotationsPerSecondSupplier
 * @param velocitySupplier
 * @param logPath
 * @param name
 * @return
 */
private static Command characterize(
        Intake intake,
        Runnable zeroVoltage,
        java.util.function.DoubleConsumer applyVoltage,
        java.util.function.DoubleSupplier angleRotationsPerSecondSupplier, 
        java.util.function.DoubleSupplier velocitySupplier,
        String logPath,
        String name) {

    List<Double> velocitySamples = new LinkedList<>();
    List<Double> accelerationSamples = new LinkedList<>();
    List<Double> gravitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();

    Timer timer = new Timer();
    final double dt = 0.02;
    final double[] lastVelocity = { 0.0 };

    return Commands.sequence(

            Commands.runOnce(() -> {
                velocitySamples.clear();
                accelerationSamples.clear();
                gravitySamples.clear();
                voltageSamples.clear();
                lastVelocity[0] = 0.0;
            }),

            Commands.run(zeroVoltage::run, intake)
                    .withTimeout(FF_START_DELAY),

            Commands.runOnce(timer::restart),

            Commands.run(() -> {

                double voltage = timer.get() * FF_RAMP_RATE;
                applyVoltage.accept(voltage);

                double velocity = velocitySupplier.getAsDouble();
                double acceleration = (velocity - lastVelocity[0]) / dt;
                lastVelocity[0] = velocity;

                double angleRotPerSec = angleRotationsPerSecondSupplier.getAsDouble();
                double angleRad = RotationsPerSecond.of(angleRotPerSec).in(RadiansPerSecond);
                double gravity = Math.cos(angleRad);

                if (Math.abs(velocity) > 5.0) {

                    velocitySamples.add(velocity);
                    accelerationSamples.add(acceleration);
                    gravitySamples.add(gravity);
                    voltageSamples.add(voltage);
                }

            }, intake)

            .finallyDo(() -> {

                int n = velocitySamples.size();
                if (n < 25) {
                    System.out.println("Not enough valid samples!");
                    return;
                }

                double s1=0,sW=0,sA=0,sG=0;
                double sWW=0,sAA=0,sGG=0;
                double sWA=0,sWG=0,sAG=0;
                double sV=0,sVW=0,sVA=0,sVG=0;

                for (int i = 0; i < n; i++) {

                    double w = velocitySamples.get(i);
                    double a = accelerationSamples.get(i);
                    double g = gravitySamples.get(i);
                    double v = voltageSamples.get(i);

                    s1++;
                    sW += w;
                    sA += a;
                    sG += g;

                    sWW += w*w;
                    sAA += a*a;
                    sGG += g*g;

                    sWA += w*a;
                    sWG += w*g;
                    sAG += a*g;

                    sV += v;
                    sVW += v*w;
                    sVA += v*a;
                    sVG += v*g;
                }

                double[][] m = {
                        { s1,  sW,  sA,  sG  },
                        { sW,  sWW, sWA, sWG },
                        { sA,  sWA, sAA, sAG },
                        { sG,  sWG, sAG, sGG }
                };

                double[] b = { sV, sVW, sVA, sVG };

                double[] gains = solve4x4(m, b);

                double kS = gains[0];
                double kV = gains[1];
                double kA = gains[2];
                double kG = gains[3];

                double kP = 0.2 * (1.0 / kV);
                double kD = kA * kP;

                NumberFormat formatter = new DecimalFormat("#0.00000");

                System.out.println("********** " + name + " Intake Pivot Characterization **********");
                System.out.println("\tkS: " + formatter.format(kS));
                System.out.println("\tkV: " + formatter.format(kV));
                System.out.println("\tkA: " + formatter.format(kA));
                System.out.println("\tkG: " + formatter.format(kG));
                System.out.println("\tkP: " + formatter.format(kP));
                System.out.println("\tkD: " + formatter.format(kD));

                Logger.recordOutput(logPath + "/kS", kS);
                Logger.recordOutput(logPath + "/kV", kV);
                Logger.recordOutput(logPath + "/kA", kA);
                Logger.recordOutput(logPath + "/kG", kG);
                Logger.recordOutput(logPath + "/kP", kP);
                Logger.recordOutput(logPath + "/kD", kD);
            }));
}

    public static Command feedforwardCharacterization_IntakeVelocity(Intake intake) {
        return characterize(
                intake,
                () -> intake.runCharacterization_IntakeVelocity(0.0),
                intake::runCharacterization_IntakeVelocity,
                intake::getFFCharacterizationVelocity_Intake,
                "Intake/Characterization/Intake",
                "Roller");
    }

    public static Command feedforwardCharacterization_IntakePosition(Intake intake) {
        return characterize(
                intake,
                () -> intake.runCharacterization_IntakeVelocity(0.0),
                intake::runCharacterization_IntakeVelocity,
                intake::getFFCharacterizationPosition_Intake,
                intake::getFFCharacterizationVelocity_Intake,
                "Intake/Characterization/Intake",
                "Pivot");
    }

    public static Command characterizeForAll(Intake intake) {
        return feedforwardCharacterization_IntakeVelocity(intake);
    }
}
