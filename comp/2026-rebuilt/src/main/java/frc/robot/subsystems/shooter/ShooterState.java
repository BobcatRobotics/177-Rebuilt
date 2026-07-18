package frc.robot.subsystems.shooter;

import org.bobcatrobotics.Framework.StateMachine.SubsystemState;
import org.bobcatrobotics.Util.Interpolators.TripleOutputInterpolator;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.RobotInfo;

public enum ShooterState  implements SubsystemState{
    IDLE(0.0, 0.0),
    MANUAL_SPINUP(1.0, 0.0),
    MANUL_SHOOT(1.0, 0.5),

    // Uses interpolation instead of fixed values
    INTERPOLATED();

    private final double rollerSpeed;
    private final double position;
    private final boolean interpolated;
    public TripleOutputInterpolator interpolator = new TripleOutputInterpolator(
      Constants.ShooterConstants.ValuesOfKnownShots.distance,
      Constants.ShooterConstants.ValuesOfKnownShots.carwashSpeed,
      Constants.ShooterConstants.ValuesOfKnownShots.dumperSpeed,
      Constants.ShooterConstants.ValuesOfKnownShots.hoodPosition,
      false);
    public TripleOutputInterpolator passingInterpolator = new TripleOutputInterpolator(
      Constants.ShooterConstants.PassingValuesOfKnownShots.distance,
      Constants.ShooterConstants.PassingValuesOfKnownShots.carwashSpeed,
      Constants.ShooterConstants.PassingValuesOfKnownShots.dumperSpeed,
      Constants.ShooterConstants.PassingValuesOfKnownShots.hoodPosition,
      false);
    ShooterState(double rollerSpeed, double position) {
        this.rollerSpeed = rollerSpeed;
        this.position = position;
        this.interpolated = false;
    }

    ShooterState() {
        double hubDistance = RobotInfo.getInstance().hubDistance;
        this.rollerSpeed = interpolator.getAsList(hubDistance).get(1);
        this.position = interpolator.getAsList(hubDistance).get(2);
        this.interpolated = true;
    }

    public void onEnter() {
        Logger.recordOutput("ShooterState", "Entered " + getStateName());
    }
    
    public void execute() {
        Logger.recordOutput("ShooterState", "In " + getStateName());
    }

    public void onExit() {
        Logger.recordOutput("ShooterState", "Exited " + getStateName());
    }


    public boolean isInterpolated() {
        return interpolated;
    }

    public double getPosition() {
        return position;
    }

    public double getRollerSpeed() {
        return rollerSpeed;
    }
    public String getStateName() {
        return name();
    }
}