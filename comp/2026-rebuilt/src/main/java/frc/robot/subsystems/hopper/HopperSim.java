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
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.subsystems.hopper.Modules.ModuleConfigurator;

public class HopperSim implements HopperIO {
    private TalonFX hopperMotor;
    private TalonFXSimState hopperMotorState;
    private FlywheelSim m_motorSimModel;
    public ModuleConfigurator hopperConfig;
    private final VelocityTorqueCurrentFOC topRequestVelocity = new VelocityTorqueCurrentFOC(0);

    private StatusSignal<AngularVelocity> velocityOfHopperTopRPS;
    private StatusSignal<Current> statorCurrentOfHopperTopAmps;
    private StatusSignal<Voltage> outputOfHopperTopVolts;
    private StatusSignal<AngularAcceleration> accelerationOfHopperTop;

    HopperState currentState;

    public HopperSim() {

        // Flywheel Configuration
        Gains topMotorGains = new Gains.Builder()
                .kP(Constants.HopperConstants.Top.kHopperP)
                .kI(Constants.HopperConstants.Top.kHopperI)
                .kD(Constants.HopperConstants.Top.kHopperD)
                .kS(Constants.HopperConstants.Top.kHopperS)
                .kV(Constants.HopperConstants.Top.kHopperV)
                .kA(Constants.HopperConstants.Top.kHopperA).build();

        DCMotor motorModel = DCMotor.getKrakenX60Foc(1);
        m_motorSimModel = new FlywheelSim(LinearSystemId.createFlywheelSystem(motorModel, .00003, 1), motorModel);

        setupTopMotor(topMotorGains);
        hopperMotorState = hopperMotor.getSimState();

        currentState = HopperState.IDLE;
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
        runMotors();

    }

    public void simulationPeriodic() {
        // Get Motor Output Voltage
        hopperMotorState = hopperMotor.getSimState();
        double motorVoltage = hopperMotorState.getMotorVoltage();
        // Feed Into Physics Simulation
        m_motorSimModel.setInputVoltage(motorVoltage);
        // Udpate SIM ( 20ms loop )
        m_motorSimModel.update(0.02);
        // get voltage ( rad/sec -> rotations/sec)
        double velocityRadPerSec = m_motorSimModel.getAngularVelocityRadPerSec();
        double velocityRotPerSec = velocityRadPerSec / (2 * Math.PI);
        // PUSH intop the TalonFX simulated Sensor the value
        hopperMotorState.setRotorVelocity(-velocityRotPerSec);
    }

    public void setState(HopperState state) {
        currentState = state;
    }

    public void runMotors() {
        hopperMotor.setControl(topRequestVelocity.withVelocity(currentState.getRollerSpeed()));
    }

    public void stop() {
        hopperMotor.stopMotor();
    }
}
