package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOTalonFXSim implements IntakeIO {

    private final TalonFX leftRollerMotor;
    private final TalonFX rightRollerMotor;
    private final TalonFX pivotMotor;

    private final TalonFXSimState leftRollerSimState;
    private final TalonFXSimState rightRollerSimState;
    private final TalonFXSimState pivotSimState;

    private final DCMotorSim leftRollerSim;
    private final DCMotorSim rightRollerSim;
    private final DCMotorSim pivotSim;

    private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0);
    private final PositionVoltage pivotPositionRequest = new PositionVoltage(0);

    public final double MOI = 0.001;
    public final double GEAR_RATIO = 1.0;


    public IntakeIOTalonFXSim(
            int leftRollerDeviceId,
            int rightRollerDeviceId,
            int pivotDeviceId) {

        leftRollerMotor = new TalonFX(leftRollerDeviceId);

        rightRollerMotor = new TalonFX(rightRollerDeviceId);

        pivotMotor = new TalonFX(pivotDeviceId);

        leftRollerSimState = leftRollerMotor.getSimState();

        rightRollerSimState = rightRollerMotor.getSimState();

        pivotSimState = pivotMotor.getSimState();

        DCMotor rollerMotor = DCMotor.getFalcon500(1);

        DCMotor pivotMotorModel = DCMotor.getFalcon500(1);

        leftRollerSim = createMotorSim(
                rollerMotor,
                this.MOI,
                this.GEAR_RATIO);

        rightRollerSim = createMotorSim(
                rollerMotor,
                this.MOI,
                this.GEAR_RATIO);

        pivotSim = createMotorSim(
                pivotMotorModel,
                this.MOI,
                this.GEAR_RATIO);
    }

    private DCMotorSim createMotorSim(
            DCMotor motor,
            double moi,
            double gearRatio) {

        return new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        motor,
                        moi,
                        gearRatio),
                motor);
    }

    // =================================================
    // ROLLERS
    // =================================================

    @Override
    public void setRollerVelocity(double rps) {

        leftRollerMotor.setControl(
                rollerVelocityRequest
                        .withVelocity(rps));

        rightRollerMotor.setControl(
                rollerVelocityRequest
                        .withVelocity(rps));
    }

    @Override
    public void stopRollers() {

        leftRollerMotor.stopMotor();
        rightRollerMotor.stopMotor();
    }

    @Override
    public double getLeftRollerVelocityRPS() {

        return leftRollerSim
                .getAngularVelocityRadPerSec()
                / (2.0 * Math.PI);
    }

    @Override
    public double getRightRollerVelocityRPS() {

        return rightRollerSim
                .getAngularVelocityRadPerSec()
                / (2.0 * Math.PI);
    }

    @Override
    public double getRollerAverageVelocityRPS() {

        return (getLeftRollerVelocityRPS() + getRightRollerVelocityRPS()) / 2.0;
    }

    // PIVOT
    @Override
    public void setPivotAngle(double angleDegrees) {

        pivotMotor.setControl(
                pivotPositionRequest.withPosition(
                        angleDegrees / 360.0));
    }

    @Override
    public void stopPivot() {

        pivotMotor.stopMotor();
    }

    @Override
    public double getPivotAngleDegrees() {

        return pivotSim
                .getAngularPositionRotations()
                * 360.0;
    }

    @Override
    public boolean isPivotAtTarget(
            double targetAngleDegrees) {

        return Math.abs(
                getPivotAngleDegrees()
                        - targetAngleDegrees) < IntakeConstants.PivotConstants.ANGLE_TOLERANCE_DEGREES;
    }

    // =================================================
    // SIMULATION
    // =================================================

    @Override
    public void simulationPeriodic() {

        updateRollerSimulation(
                leftRollerSim,
                leftRollerSimState);

        updateRollerSimulation(
                rightRollerSim,
                rightRollerSimState);

        updatePivotSimulation();
    }

    private void updateRollerSimulation(
            DCMotorSim simulation,
            TalonFXSimState simState) {

        simulation.setInputVoltage(
                simState.getMotorVoltage());

        simulation.update(0.020);

        simState.setRawRotorPosition(
                simulation
                        .getAngularPositionRotations());

        simState.setRotorVelocity(
                simulation
                        .getAngularVelocityRadPerSec()
                        / (2.0 * Math.PI));
    }

    private void updatePivotSimulation() {

        pivotSim.setInputVoltage(
                pivotSimState.getMotorVoltage());

        pivotSim.update(0.020);

        pivotSimState.setRawRotorPosition(
                pivotSim
                        .getAngularPositionRotations());

        pivotSimState.setRotorVelocity(
                pivotSim
                        .getAngularVelocityRadPerSec()
                        / (2.0 * Math.PI));
    }
}