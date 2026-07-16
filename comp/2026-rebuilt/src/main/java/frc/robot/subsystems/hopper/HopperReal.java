package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.bobcatrobotics.Util.Tunables.Gains;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.hopper.Modules.ModuleConfigurator;

public class HopperReal implements HopperIO {
    private TalonFX hopperMotor;
    public ModuleConfigurator hopperConfig;
    private final VelocityTorqueCurrentFOC topRequestVelocity = new VelocityTorqueCurrentFOC(0);

    private StatusSignal<AngularVelocity> velocityOfHopperTopRPS;
    private StatusSignal<Current> statorCurrentOfHopperTopAmps;
    private StatusSignal<Voltage> outputOfHopperTopVolts;
    private StatusSignal<AngularAcceleration> accelerationOfHopperTop;

    public HopperReal() {

        // Flywheel Configuration
        Gains topMotorGains = new Gains.Builder()
                .kP(Constants.HopperConstants.Top.kHopperP)
                .kI(Constants.HopperConstants.Top.kHopperI)
                .kD(Constants.HopperConstants.Top.kHopperD)
                .kS(Constants.HopperConstants.Top.kHopperS)
                .kV(Constants.HopperConstants.Top.kHopperV)
                .kA(Constants.HopperConstants.Top.kHopperA).build();

        setupTopMotor(topMotorGains);

    }

    public void setupTopMotor(Gains g) {
        hopperConfig = new ModuleConfigurator(g.toSlot0Configs(),
                Constants.HopperConstants.Top.hopperMotorId,
                Constants.HopperConstants.Top.isInverted,
                Constants.HopperConstants.Top.isCoast,
                Constants.HopperConstants.Top.hopperCurrentLimit);
        hopperMotor = new TalonFX(hopperConfig.getMotorInnerId(), new CANBus("rio"));
        hopperConfig.configureMotor(hopperMotor, g);
        if (Constants.lowTelemetryMode) {
            velocityOfHopperTopRPS = hopperMotor.getVelocity();
            statorCurrentOfHopperTopAmps = hopperMotor.getStatorCurrent();
            hopperConfig.configureSignals(hopperMotor, 50.0, velocityOfHopperTopRPS,
                    statorCurrentOfHopperTopAmps);
        } else {
            velocityOfHopperTopRPS = hopperMotor.getVelocity();
            statorCurrentOfHopperTopAmps = hopperMotor.getStatorCurrent();
            outputOfHopperTopVolts = hopperMotor.getMotorVoltage();
            accelerationOfHopperTop = hopperMotor.getAcceleration();
            hopperConfig.configureSignals(hopperMotor, 50.0, velocityOfHopperTopRPS,
                    statorCurrentOfHopperTopAmps, accelerationOfHopperTop, accelerationOfHopperTop);
        }
    }

    public void updateInputs(HopperIOInputs inputs) {
        if (Constants.lowTelemetryMode) {
            lowTelemetry(inputs);
        } else {
            highTelemetry(inputs);
        }

    }

    public void highTelemetry(HopperIOInputs inputs) {
        BaseStatusSignal.refreshAll(accelerationOfHopperTop, outputOfHopperTopVolts);
        inputs.accelerationOfHopperTop = accelerationOfHopperTop.getValue()
                .in(RotationsPerSecondPerSecond);
        inputs.outputOfHopperTopVolts = outputOfHopperTopVolts.getValue().in(Volts);
        lowTelemetry(inputs);
    }

    public void lowTelemetry(HopperIOInputs inputs) {
        BaseStatusSignal.refreshAll(velocityOfHopperTopRPS, statorCurrentOfHopperTopAmps);
        inputs.velocityOfHopperTopRPS = velocityOfHopperTopRPS.getValue().in(Rotation.per(Minute));
        inputs.statorCurrentOfHopperTopAmps = statorCurrentOfHopperTopAmps.getValue().in(Amps);
        inputs.hopperTopConnected = hopperMotor.isConnected();
        inputs.torqueCurrentHopperTopAmps = hopperMotor.getTorqueCurrent().getValue().in(Amps);
    }

    public void periodic() {

    }

    public void simulationPeriodic() {
    }


    public void runMotors(HopperState currentState) {
        hopperMotor.setControl(topRequestVelocity.withVelocity(currentState.getRollerSpeed()));
    }

    public void stop() {
        hopperMotor.stopMotor();
    }
}
