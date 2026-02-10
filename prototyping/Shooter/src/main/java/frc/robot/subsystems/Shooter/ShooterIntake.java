package frc.robot.subsystems.Shooter;

import org.bobcatrobotics.Util.Tunables.TunableDouble;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Modules.ModuleConfigurator;

public class ShooterIntake implements ShooterIO {
    private final TalonFX shooterIntakeMotor;
    private TalonFXConfiguration shooterIntakeConfig = new TalonFXConfiguration();

    // Defines tunable values , particularly for configurations of motors ( IE PIDs
    // )
    private TunableDouble shooterIntakeMotorsPIDkP;
    private TunableDouble shooterIntakeMotorsPIDkI;
    private TunableDouble shooterIntakeMotorsPIDkD;
    private TunableDouble shooterIntakeMotorsPIDkV;
    private TunableDouble shooterIntakeMotorsPIDkS;
    private TunableDouble shooterIntakeMotorsPIDkA;
    private final VelocityTorqueCurrentFOC velIntakeRequest = new VelocityTorqueCurrentFOC(0);
    // private final DutyCycleOut velIntakeRequest = new DutyCycleOut(0);
    private StatusSignal<AngularVelocity> velocityOfIntakeRPS;
    private StatusSignal<Current> statorCurrentOfIntakeAmps;
    private StatusSignal<Voltage> outputOfIntakeVolts;
    private StatusSignal<AngularAcceleration> accelerationOfIntake;

    public ModuleConfigurator intakeConfig;
    public double mainFlywheelSetpoint = 0;
    public double backspinSetpoint = 0;

    private String name;

    public ShooterIntake(String name) {
        this.name = name;

        if (name == "Left") {
            // left
            shooterIntakeMotorsPIDkP = new TunableDouble("/Shooter/Intake/" + name + "/Intake/PID/kP",
                    Constants.ShooterConstants.kIntakeMotorkP);
            shooterIntakeMotorsPIDkI = new TunableDouble("/Shooter/Intake/" + name + "/Intake/PID/kI",
                    Constants.ShooterConstants.kIntakeMotorkI);
            shooterIntakeMotorsPIDkD = new TunableDouble("/Shooter/Intake/" + name + "/Intake/PID/kD",
                    Constants.ShooterConstants.kIntakeMotorkD);
            shooterIntakeMotorsPIDkV = new TunableDouble("/Shooter/Intake/" + name + "/Intake/PID/kV",
                    Constants.ShooterConstants.kIntakeMotorkV);
            shooterIntakeMotorsPIDkS = new TunableDouble("/Shooter/Intake/" + name + "/Intake/PID/kS",
                    Constants.ShooterConstants.kIntakeMotorkS);
            shooterIntakeMotorsPIDkA = new TunableDouble("/Shooter/Intake/" + name + "/Intake/PID/kA",
                    Constants.ShooterConstants.kIntakeMotorkA);
        } else {

            // left
            shooterIntakeMotorsPIDkP = new TunableDouble("/Shooter/Intake/" + name + "/Intake/PID/kP",
                    Constants.ShooterConstants.kIntakeMotorkP);
            shooterIntakeMotorsPIDkI = new TunableDouble("/Shooter/Intake/" + name + "/Intake/PID/kI",
                    Constants.ShooterConstants.kIntakeMotorkI);
            shooterIntakeMotorsPIDkD = new TunableDouble("/Shooter/Intake/" + name + "/Intake/PID/kD",
                    Constants.ShooterConstants.kIntakeMotorkD);
            shooterIntakeMotorsPIDkV = new TunableDouble("/Shooter/Intake/" + name + "/Intake/PID/kV",
                    Constants.ShooterConstants.kIntakeMotorkV);
            shooterIntakeMotorsPIDkS = new TunableDouble("/Shooter/Intake/" + name + "/Intake/PID/kS",
                    Constants.ShooterConstants.kIntakeMotorkS);
            shooterIntakeMotorsPIDkA = new TunableDouble("/Shooter/Intake/" + name + "/Intake/PID/kA",
                    Constants.ShooterConstants.kIntakeMotorkA);
        }
        Slot0Configs intakeConfigLeft = new Slot0Configs();
        intakeConfigLeft.kP = shooterIntakeMotorsPIDkP.get();
        intakeConfigLeft.kI = shooterIntakeMotorsPIDkI.get();
        intakeConfigLeft.kD = shooterIntakeMotorsPIDkD.get();
        intakeConfigLeft.kV = shooterIntakeMotorsPIDkV.get();
        intakeConfigLeft.kS = shooterIntakeMotorsPIDkS.get();
        intakeConfigLeft.kA = shooterIntakeMotorsPIDkA.get();
        intakeConfig = new ModuleConfigurator(intakeConfigLeft, Constants.ShooterConstants.Left.BackspinIDLeft, false,
                true, 40);
        shooterIntakeMotor = new TalonFX(intakeConfig.getMotorInnerId(), new CANBus("rio"));
        configureShooterIntake();

        // Apply to signals
        velocityOfIntakeRPS = shooterIntakeMotor.getVelocity();
        accelerationOfIntake = shooterIntakeMotor.getAcceleration();
        statorCurrentOfIntakeAmps = shooterIntakeMotor.getStatorCurrent();
        outputOfIntakeVolts = shooterIntakeMotor.getMotorVoltage();
        // Set polling frequency and optimizations
        BaseStatusSignal.setUpdateFrequencyForAll(50, velocityOfIntakeRPS);
        BaseStatusSignal.setUpdateFrequencyForAll(50, statorCurrentOfIntakeAmps);
        BaseStatusSignal.setUpdateFrequencyForAll(50, outputOfIntakeVolts);
        BaseStatusSignal.setUpdateFrequencyForAll(50, accelerationOfIntake);
        shooterIntakeMotor.optimizeBusUtilization();

    }

    public String getName() {
        return name;
    }

    /**
     * Configures the left and right motors of the "main" flywheel these are the
     * forward bottom most motors.
     */
    public void configureShooterIntake() {
        Slot0Configs slot0 = new Slot0Configs();
        // left
        slot0.kP = shooterIntakeMotorsPIDkP.get();
        slot0.kI = shooterIntakeMotorsPIDkI.get();
        slot0.kD = shooterIntakeMotorsPIDkD.get();
        slot0.kV = shooterIntakeMotorsPIDkV.get();
        slot0.kS = shooterIntakeMotorsPIDkS.get();

        // Top motor configurations
        shooterIntakeMotor.getConfigurator().apply(shooterIntakeConfig); // reset to default
        if (intakeConfig.isInnerInverted()) {
            shooterIntakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            shooterIntakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }
        if (intakeConfig.isCoast()) {
            shooterIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        } else {
            shooterIntakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }
        shooterIntakeConfig.Slot0 = slot0;
        shooterIntakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterIntakeConfig.CurrentLimits.StatorCurrentLimit = intakeConfig.getCurrentLimit();
        shooterIntakeMotor.getConfigurator().apply(shooterIntakeConfig);
    }

    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                velocityOfIntakeRPS,
                statorCurrentOfIntakeAmps,
                outputOfIntakeVolts,
                accelerationOfIntake);

        inputs.velocityOfIntakeWheelMotorRPS = velocityOfIntakeRPS.getValue().in(Rotation.per(Seconds));
        inputs.velocityOfIntakeWheelMotorRPSError = mainFlywheelSetpoint - inputs.velocityOfIntakeWheelMotorRPS;
        inputs.outputOfIntakeWheelInner = outputOfIntakeVolts.getValue().in(Volts);
        inputs.accelerationOfIntakeWheelMotor = accelerationOfIntake.getValue().in(RotationsPerSecondPerSecond);
        inputs.mainIntakeStatorCurrent = statorCurrentOfIntakeAmps.getValue().in(Amps);
        inputs.intakeConnected = shooterIntakeMotor.isConnected();
    }

    public void setVelocity(ShooterState desiredState) {
        setVelocity(desiredState.getFlywheelSpeed(),
                desiredState.getBackspinSpeed(), desiredState.getIntakeSpeed());
    }

    public void setVelocity(double shooterFlywheelSpeed, double shooterBackspinSpeed, double ShooterIntakeSpeed) {
        setIntakeSpeed(ShooterIntakeSpeed);
    }

    public void setIntakeSpeed(double shooterFlywheelSpeed) {
        mainFlywheelSetpoint = shooterFlywheelSpeed;
        shooterIntakeMotor.setControl(velIntakeRequest.withVelocity(shooterFlywheelSpeed));
    }

    public void holdPosition() {
    }

    public void stopMainWheel() {
        mainFlywheelSetpoint = 0;
        shooterIntakeMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // left
        if (shooterIntakeMotorsPIDkP.hasChanged()
                || shooterIntakeMotorsPIDkI.hasChanged()
                || shooterIntakeMotorsPIDkD.hasChanged()
                || shooterIntakeMotorsPIDkS.hasChanged()
                || shooterIntakeMotorsPIDkV.hasChanged()
                || shooterIntakeMotorsPIDkA.hasChanged()) {
            configureShooterIntake();
        }
    }

    public void simulationPeriodic() {
    }
}