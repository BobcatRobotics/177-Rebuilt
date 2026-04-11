package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.bobcatrobotics.Util.CANDeviceDetails;
import org.bobcatrobotics.Util.Interpolators.TripleOutputInterpolator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Carwash.CarwashState;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Shooter.ShooterState;

public class RobotState {
  public CharacterizationType characterizationType = CharacterizationType.DRIVE;
  public Alliance alliance;
  private static RobotState instance;
  public double vx = 0.0;
  public double vy = 0.0;

  public static RobotState getInstance() {
    if (instance == null)
    {
      instance = new RobotState();
    }
    return instance;
  }

  // Save Odometry,
  // Save Swerve Module Details
  // Save Vision Tags

  // Save Shooter State
  private ShooterState.State desiredShooterStateType = ShooterState.State.IDLE;
  private ShooterState desiredShooterState = new ShooterState();

  public ShooterState.State getDesiredShooterStateType() {
    return desiredShooterStateType;
  }

  public ShooterState getShooterState() {
    return desiredShooterState;
  }

    // Save Carwash State
  private CarwashState.State desiredCarwashStateType = CarwashState.State.IDLE;
  private CarwashState desiredCarwashState = new CarwashState();

  public CarwashState.State getDesiredCarwashStateType() {
    return desiredCarwashStateType;
  }

  public CarwashState getCarwashState() {
    return desiredCarwashState;
  }

  // Save Hopper State
  private HopperState.State desiredHopperStateType = HopperState.State.IDLE;
  private HopperState desiredHopperState  = new HopperState();

  public HopperState.State getDesiredHopperStateType() {
    return desiredHopperStateType;
  }

  public HopperState getHopperState() {
    return desiredHopperState;
  }

  // Save Intake State
  private IntakeState.State desiredIntakeStateType = IntakeState.State.IDLE;
  private IntakeState desiredIntakeState = new IntakeState();

  public IntakeState.State getDesiredIntakeStateType() {
    return desiredIntakeStateType;
  }

  public IntakeState getIntakeState() {
    return desiredIntakeState;
  }

    public TripleOutputInterpolator interpolator = new TripleOutputInterpolator(
      Constants.ShooterConstants.ValuesOfKnownShots.distance,
      Constants.ShooterConstants.ValuesOfKnownShots.carwashSpeed,
      Constants.ShooterConstants.ValuesOfKnownShots.hoodSpeed,
      Constants.ShooterConstants.ValuesOfKnownShots.mainFlyWheelSpeed,
      false);

  public Pose2d robotPose = new Pose2d();
  public boolean shooterUpToSpeed = false;
  public boolean hubInrange = false;
  public double hubDistance = 0.0;
  // used by new align command not to be confused with hubInRange;
  public boolean isRobotAlignedToHub = false;


  // CAN Device & Subsystem Devices
  public HashMap<String, List<CANDeviceDetails>> devices = new HashMap<String, List<CANDeviceDetails>>();
  public List<CANDeviceDetails> subsytemDriveDevices = new ArrayList<CANDeviceDetails>(); 
  public List<CANDeviceDetails> subsytemHopperDevices = new ArrayList<CANDeviceDetails>(); 
  public List<CANDeviceDetails> subsytemCarwashDevices = new ArrayList<CANDeviceDetails>();
  public List<CANDeviceDetails> subsytemIntakeDevices = new ArrayList<CANDeviceDetails>();
  public List<CANDeviceDetails> subsytemShooterDevices = new ArrayList<CANDeviceDetails>();
}
