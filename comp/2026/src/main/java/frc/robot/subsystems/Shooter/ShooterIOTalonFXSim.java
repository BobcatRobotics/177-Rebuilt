package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOTalonFXSim implements ShooterIO {

    private final TalonFX innerLeftMotor;
    private final TalonFX innerRightMotor;
    private final TalonFX outerLeftMotor;
    private final TalonFX outerRightMotor;

    private final TalonFXSimState innerLeftSimState;
    private final TalonFXSimState innerRightSimState;
    private final TalonFXSimState outerLeftSimState;
    private final TalonFXSimState outerRightSimState;

    private final DCMotorSim innerLeftSim;
    private final DCMotorSim innerRightSim;
    private final DCMotorSim outerLeftSim;
    private final DCMotorSim outerRightSim;

    private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

    public final double MOI = 0.001;
    public final double GEAR_RATIO = 1.0;


    public ShooterIOTalonFXSim(
            int innerLeftDeviceId,
            int innerRightDeviceId,
            int outerLeftDeviceId,
            int outerRightDeviceId) {

        innerLeftMotor = new TalonFX(innerLeftDeviceId);
        innerRightMotor = new TalonFX(innerRightDeviceId);
        outerLeftMotor = new TalonFX(outerLeftDeviceId);
        outerRightMotor = new TalonFX(outerRightDeviceId);

        innerLeftSimState = innerLeftMotor.getSimState();
        innerRightSimState = innerRightMotor.getSimState();
        outerLeftSimState = outerLeftMotor.getSimState();
        outerRightSimState = outerRightMotor.getSimState();

        DCMotor motor = DCMotor.getFalcon500(1);

        innerLeftSim = createMotorSim(motor);
        innerRightSim = createMotorSim(motor);
        outerLeftSim = createMotorSim(motor);
        outerRightSim = createMotorSim(motor);
    }

    private DCMotorSim createMotorSim( DCMotor motor) {
        return new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, this.MOI,this.GEAR_RATIO), motor);
    }

    @Override
    public void setFlywheelVelocity(double rps) {
        innerLeftMotor.setControl(velocityRequest.withVelocity(rps));
        innerRightMotor.setControl(velocityRequest.withVelocity(rps));
        outerLeftMotor.setControl(velocityRequest.withVelocity(rps));
        outerRightMotor.setControl(velocityRequest.withVelocity(rps));
    }

    @Override
    public void stopFlywheel() {
        innerLeftMotor.stopMotor();
        innerRightMotor.stopMotor();
        outerLeftMotor.stopMotor();
        outerRightMotor.stopMotor();
    }

    // ==================================================
    // VELOCITY
    // ==================================================

    @Override
    public double getLeftInnerVelocityRPS() {

        return innerLeftSim
                .getAngularVelocityRadPerSec()
                / (2.0 * Math.PI);
    }

    @Override
    public double getRightInnerVelocityRPS() {

        return innerRightSim
                .getAngularVelocityRadPerSec()
                / (2.0 * Math.PI);
    }

    @Override
    public double getLeftOuterVelocityRPS() {

        return outerLeftSim
                .getAngularVelocityRadPerSec()
                / (2.0 * Math.PI);
    }

    @Override
    public double getRightOuterVelocityRPS() {

        return outerRightSim
                .getAngularVelocityRadPerSec()
                / (2.0 * Math.PI);
    }

    // ==================================================
    // AVERAGE SHOOTER VELOCITY
    // ==================================================

    @Override
    public double getFlywheelVelocityRPS() {

        return (getLeftInnerVelocityRPS()
                + getRightInnerVelocityRPS()
                + getLeftOuterVelocityRPS()
                + getRightOuterVelocityRPS()) / 4.0;
    }

    // ==================================================
    // AT TARGET SPEED
    // ==================================================

    @Override
    public boolean atTargetSpeed(
            double targetRPS) {

        return Math.abs(
                getFlywheelVelocityRPS()
                        - targetRPS) <= ShooterConstants.SPEED_TOLERANCE_RPS;
    }

    // ==================================================
    // SIMULATION
    // ==================================================

    @Override
    public void simulationPeriodic() {

        updateMotorSimulation(
                innerLeftSim,
                innerLeftSimState);

        updateMotorSimulation(
                innerRightSim,
                innerRightSimState);

        updateMotorSimulation(
                outerLeftSim,
                outerLeftSimState);

        updateMotorSimulation(
                outerRightSim,
                outerRightSimState);
    }

    private void updateMotorSimulation(
            DCMotorSim motorSim,
            TalonFXSimState simState) {

        // ---------------------------------------------
        // Simulated battery voltage
        // ---------------------------------------------

        simState.setSupplyVoltage(12.0);

        // ---------------------------------------------
        // TalonFX determines motor voltage
        // based on the control request
        // ---------------------------------------------

        double motorVoltage = simState.getMotorVoltage();

        // ---------------------------------------------
        // Feed voltage into physics model
        // ---------------------------------------------

        motorSim.setInputVoltage(
                motorVoltage);

        // ---------------------------------------------
        // Advance physics by 20 ms
        // ---------------------------------------------

        motorSim.update(0.020);

        // ---------------------------------------------
        // Convert mechanism state back into
        // TalonFX rotor units
        // ---------------------------------------------
        simState.setRawRotorPosition(
                motorSim.getAngularPositionRotations());

        simState.setRotorVelocity(
                motorSim.getAngularVelocityRadPerSec()
                        / (2.0 * Math.PI));
    }
}