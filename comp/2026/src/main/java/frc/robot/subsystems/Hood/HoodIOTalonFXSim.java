package frc.robot.subsystems.Hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOTalonFXSim implements HoodIO {

    private final TalonFX hoodMotor;
    private final TalonFXSimState motorSimState;
    private final SingleJointedArmSim armSim;
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public static final double ARM_MOI = 0.02;
    public static final double ARM_LENGTH_METERS = 0.30;
    public static final double ARM_MASS_KG = 2.0;
    public static final double MIN_ANGLE_DEGREES = 0.0;
    public static final double MAX_ANGLE_DEGREES = 70.0;

    public HoodIOTalonFXSim(int motorId) {

        hoodMotor = new TalonFX(motorId);
        motorSimState = hoodMotor.getSimState();
        DCMotor motors = DCMotor.getFalcon500(2);
        armSim = new SingleJointedArmSim(
                motors,
                HoodConstants.GEAR_RATIO,
                SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, ARM_MASS_KG), ARM_LENGTH_METERS,
                Math.toRadians(MIN_ANGLE_DEGREES),
                Math.toRadians(MAX_ANGLE_DEGREES),
                true,
                Math.toRadians(MIN_ANGLE_DEGREES));
    }

    @Override
    public void setAngle(double angleDegrees) {
        double motorRotations = degreesToMotorRotations(angleDegrees);
        hoodMotor.setControl(positionRequest.withPosition(motorRotations));
    }

    @Override
    public void stop() {
        hoodMotor.stopMotor();
    }

    @Override
    public double getAngleDegrees() {
        return Math.toDegrees(armSim.getAngleRads());
    }

    @Override
    public boolean atTargetAngle(double targetAngleDegrees) {

        return Math.abs(getAngleDegrees() - targetAngleDegrees) <= HoodConstants.ANGLE_TOLERANCE_DEGREES;
    }

    @Override
    public double getHoodMotorVelocityRPS() {
        return hoodMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void simulationPeriodic() {
        motorSimState.setSupplyVoltage(12.0);

        double voltage = motorSimState.getMotorVoltage();
        armSim.setInputVoltage(voltage);
        armSim.update(0.020);

        double angleDegrees = getAngleDegrees();

        double motorRotations = degreesToMotorRotations(angleDegrees);

        double motorRPS = armSim.getVelocityRadPerSec() / (2.0 * Math.PI) * HoodConstants.GEAR_RATIO;

        motorSimState.setRawRotorPosition(motorRotations);

        motorSimState.setRotorVelocity(motorRPS);
    }

    private double degreesToMotorRotations(double degrees) {

        return degrees / 360.0 * HoodConstants.GEAR_RATIO;
    }
}