package frc.robot.subsystems.carwash;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
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
import frc.robot.subsystems.carwash.Modules.ModuleConfigurator;

public class CarwashSim implements CarwashIO {
    private TalonFX shooterIntakeMotor;
    private TalonFXSimState shooterIntakeMotorState;
    private FlywheelSim m_motorSimModel;
    public ModuleConfigurator intakeWheelConfig;

    private VelocityTorqueCurrentFOC velIntakeRequest = new VelocityTorqueCurrentFOC(0);
    private StatusSignal<AngularVelocity> velocityOfIntakeRPS;
    private StatusSignal<Current> statorCurrentOfIntakeAmps;
    private StatusSignal<Voltage> outputOfIntakeVolts;
    private StatusSignal<AngularAcceleration> accelerationOfIntake;
    CarwashState currentState;

    public CarwashSim() {
        Gains intakeGains = new Gains.Builder()
                .kP(Constants.CarwashConstants.SharedIntake.kIntakeMotorkP)
                .kI(Constants.CarwashConstants.SharedIntake.kIntakeMotorkI)
                .kD(Constants.CarwashConstants.SharedIntake.kIntakeMotorkD)
                .kS(Constants.CarwashConstants.SharedIntake.kIntakeMotorkS)
                .kV(Constants.CarwashConstants.SharedIntake.kIntakeMotorkV)
                .kA(Constants.CarwashConstants.SharedIntake.kIntakeMotorkA).build();

    DCMotor motorModel = DCMotor.getKrakenX60Foc(1);
    m_motorSimModel = new FlywheelSim(LinearSystemId.createFlywheelSystem(motorModel, .00003, 1),motorModel);

    setupIntake(intakeGains);
    shooterIntakeMotorState = shooterIntakeMotor.getSimState();

        currentState = CarwashState.IDLE;
    }

    public void setupIntake(Gains g) {
        intakeWheelConfig = new ModuleConfigurator(g.toSlot0Configs(),
                Constants.CarwashConstants.SharedIntake.intakeIDLeft,
                Constants.CarwashConstants.SharedIntake.isInverted,
                Constants.CarwashConstants.SharedIntake.isCoast,
                Constants.CarwashConstants.SharedIntake.statorCurrentLimit,
                Constants.CarwashConstants.SharedIntake.supplyCurrentLimit);
        shooterIntakeMotor = new TalonFX(intakeWheelConfig.getMotorInnerId(), new CANBus("rio"));
        intakeWheelConfig.configureMotor(shooterIntakeMotor, g);
        if (Constants.lowTelemetryMode) {
            velocityOfIntakeRPS = shooterIntakeMotor.getVelocity();
            statorCurrentOfIntakeAmps = shooterIntakeMotor.getStatorCurrent();
            intakeWheelConfig.configureSignals(shooterIntakeMotor, 50.0, velocityOfIntakeRPS,
                    statorCurrentOfIntakeAmps);
        } else {
            velocityOfIntakeRPS = shooterIntakeMotor.getVelocity();
            statorCurrentOfIntakeAmps = shooterIntakeMotor.getStatorCurrent();
            outputOfIntakeVolts = shooterIntakeMotor.getMotorVoltage();
            accelerationOfIntake = shooterIntakeMotor.getAcceleration();
            intakeWheelConfig.configureSignals(shooterIntakeMotor, 50.0, velocityOfIntakeRPS,
                    statorCurrentOfIntakeAmps, outputOfIntakeVolts, accelerationOfIntake);
        }
    }

    public void updateInputs(CarwashIOInputs inputs) {
        if (Constants.lowTelemetryMode) {
            lowTelemetry(inputs);
        } else {
            highTelemetry(inputs);
        }
    }

    public void highTelemetry(CarwashIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                accelerationOfIntake,
                outputOfIntakeVolts);
        inputs.accelerationOfIntake = accelerationOfIntake.getValue()
                .in(RotationsPerSecondPerSecond);
        inputs.outputOfIntakeVolts = outputOfIntakeVolts.getValue().in(Volts);
        lowTelemetry(inputs);
    }

    public void lowTelemetry(CarwashIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                velocityOfIntakeRPS, statorCurrentOfIntakeAmps);
        inputs.velocityOfIntakeRPS = velocityOfIntakeRPS.getValue().in(Rotations.per(Seconds));
        inputs.statorCurrentOfIntakeAmps = statorCurrentOfIntakeAmps.getValue().in(Amps);
        inputs.shooterIntakeMotorConnected = shooterIntakeMotor.isConnected();
        inputs.torqueCurrentCarwashAmps = shooterIntakeMotor.getTorqueCurrent().getValue().in(Amps);
    }

    public void periodic() {
        runMotors();

    }

    public void simulationPeriodic() {
        // Get Motor Output Voltage
        shooterIntakeMotorState = shooterIntakeMotor.getSimState();
        double motorVoltage = shooterIntakeMotorState.getMotorVoltage();
        // Feed Into Physics Simulation
        m_motorSimModel.setInputVoltage(motorVoltage);
        // Udpate SIM ( 20ms loop )
        m_motorSimModel.update(0.02);
        // get voltage ( rad/sec -> rotations/sec)
        double velocityRadPerSec = m_motorSimModel.getAngularVelocityRadPerSec();
        double velocityRotPerSec = velocityRadPerSec / (2 * Math.PI);
        // PUSH intop the TalonFX simulated Sensor the value
        shooterIntakeMotorState.setRotorVelocity(-velocityRotPerSec);
    }

    public void setState(CarwashState state) {
        currentState = state;
    }

    public void runMotors() {
        shooterIntakeMotor.setControl(velIntakeRequest.withVelocity(currentState.getCarwashSpeed()));
    }

    public void stop() {
        shooterIntakeMotor.stopMotor();
    }
}
