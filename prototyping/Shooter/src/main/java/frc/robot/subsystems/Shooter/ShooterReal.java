package frc.robot.subsystems.Shooter;

import org.bobcatrobotics.Util.Tunables.TunableDouble;
import com.ctre.phoenix6.configs.Slot0Configs;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Modules.Configurator;
import frc.robot.subsystems.Shooter.Modules.ShooterModuleSingle;

public class ShooterReal implements ShooterIO {

  private ShooterModuleSingle singleMotorShooterLeft;
  private ShooterModuleSingle singleMotorShooterRight;

  // Defines tunable values , particularly for configurations of motors ( IE PIDs
  // )
  private TunableDouble shooterLeftMainMotorsPIDkP;
  private TunableDouble shooterLeftMainMotorsPIDkI;
  private TunableDouble shooterLeftMainMotorsPIDkD;
  private TunableDouble shooterLeftMainMotorsPIDkV;
  private TunableDouble shooterLeftMainMotorsPIDkS;
  private TunableDouble shooterLeftMainMotorsPIDkA;
  private TunableDouble shooterLeftIntakeMotorsPIDkP;
  private TunableDouble shooterLeftIntakeMotorsPIDkI;
  private TunableDouble shooterLeftIntakeMotorsPIDkD;
  private TunableDouble shooterLeftIntakeMotorsPIDkV;
  private TunableDouble shooterLeftIntakeMotorsPIDkS;
  private TunableDouble shooterLeftIntakeMotorsPIDkA;
  private TunableDouble shooterLeftBackspinMotorsPIDkP;
  private TunableDouble shooterLeftBackspinMotorsPIDkI;
  private TunableDouble shooterLeftBackspinMotorsPIDkD;
  private TunableDouble shooterLeftBackspinMotorsPIDkV;
  private TunableDouble shooterLeftBackspinMotorsPIDkS;
  private TunableDouble shooterLeftBackspinMotorsPIDkA;

  private TunableDouble shooterRightMainMotorsPIDkP;
  private TunableDouble shooterRightMainMotorsPIDkI;
  private TunableDouble shooterRightMainMotorsPIDkD;
  private TunableDouble shooterRightMainMotorsPIDkV;
  private TunableDouble shooterRightMainMotorsPIDkS;
  private TunableDouble shooterRightMainMotorsPIDkA;
  private TunableDouble shooterRightIntakeMotorsPIDkP;
  private TunableDouble shooterRightIntakeMotorsPIDkI;
  private TunableDouble shooterRightIntakeMotorsPIDkD;
  private TunableDouble shooterRightIntakeMotorsPIDkV;
  private TunableDouble shooterRightIntakeMotorsPIDkS;
  private TunableDouble shooterRightIntakeMotorsPIDkA;
  private TunableDouble shooterRightBackspinMotorsPIDkP;
  private TunableDouble shooterRightBackspinMotorsPIDkI;
  private TunableDouble shooterRightBackspinMotorsPIDkD;
  private TunableDouble shooterRightBackspinMotorsPIDkV;
  private TunableDouble shooterRightBackspinMotorsPIDkS;
  private TunableDouble shooterRightBackspinMotorsPIDkA;

  private Configurator leftFlywheelConfig;
  private Configurator leftBackspinConfig;
  private Configurator leftIntakeConfig;
  private Configurator rightFlywheelConfig;
  private Configurator rightBackspinConfig;
  private Configurator rightIntakeConfig;

  public ShooterReal() {
    // left
    shooterLeftMainMotorsPIDkP = new TunableDouble("/Shooter/MainMotor/Left/PID/kP",
        Constants.ShooterConstants.LeftShooterConstants.kshooterMainkP);
    shooterLeftMainMotorsPIDkI = new TunableDouble("/Shooter/MainMotor/Left/PID/kI",
        Constants.ShooterConstants.LeftShooterConstants.kshooterMainkI);
    shooterLeftMainMotorsPIDkD = new TunableDouble("/Shooter/MainMotor/Left/PID/kD",
        Constants.ShooterConstants.LeftShooterConstants.kshooterMainkD);
    shooterLeftMainMotorsPIDkV = new TunableDouble("/Shooter/MainMotor/Left/PID/kV",
        Constants.ShooterConstants.LeftShooterConstants.kshooterMainkV);
    shooterLeftMainMotorsPIDkS = new TunableDouble("/Shooter/MainMotor/Left/PID/kS",
        Constants.ShooterConstants.LeftShooterConstants.kshooterMainkS);
    shooterLeftMainMotorsPIDkA = new TunableDouble("/Shooter/MainMotor/Left/PID/kA",
        Constants.ShooterConstants.LeftShooterConstants.kshooterMainkA);
    shooterLeftIntakeMotorsPIDkP = new TunableDouble("/Shooter/Intake/Left/PID/kP",
        Constants.ShooterConstants.LeftShooterConstants.shooterIntakeMotorkP);
    shooterLeftIntakeMotorsPIDkI = new TunableDouble("/Shooter/Intake/Left/PID/kI",
        Constants.ShooterConstants.LeftShooterConstants.shooterIntakeMotorkI);
    shooterLeftIntakeMotorsPIDkD = new TunableDouble("/Shooter/Intake/Left/PID/kD",
        Constants.ShooterConstants.LeftShooterConstants.shooterIntakeMotorkD);
    shooterLeftIntakeMotorsPIDkV = new TunableDouble("/Shooter/Intake/Left/PID/kV",
        Constants.ShooterConstants.LeftShooterConstants.shooterIntakeMotorkV);
    shooterLeftIntakeMotorsPIDkS = new TunableDouble("/Shooter/Intake/Left/PID/kS",
        Constants.ShooterConstants.LeftShooterConstants.shooterIntakeMotorkS);
    shooterLeftIntakeMotorsPIDkA = new TunableDouble("/Shooter/Intake/Left/PID/kA",
        Constants.ShooterConstants.LeftShooterConstants.shooterIntakeMotorkA);
    shooterLeftBackspinMotorsPIDkP = new TunableDouble("/Shooter/BackSpin/Left/PID/kP",
        Constants.ShooterConstants.LeftShooterConstants.shooterBackspinMotorLeftMountedkP);
    shooterLeftBackspinMotorsPIDkI = new TunableDouble("/Shooter/BackSpin/Left/PID/kI",
        Constants.ShooterConstants.LeftShooterConstants.shooterBackspinMotorLeftMountedkI);
    shooterLeftBackspinMotorsPIDkD = new TunableDouble("/Shooter/BackSpin/Left/PID/kD",
        Constants.ShooterConstants.LeftShooterConstants.shooterBackspinMotorLeftMountedkD);
    shooterLeftBackspinMotorsPIDkV = new TunableDouble("/Shooter/BackSpin/Left/PID/kV",
        Constants.ShooterConstants.LeftShooterConstants.shooterBackspinMotorLeftMountedkV);
    shooterLeftBackspinMotorsPIDkS = new TunableDouble("/Shooter/BackSpin/Left/PID/kS",
        Constants.ShooterConstants.LeftShooterConstants.shooterBackspinMotorLeftMountedkS);
    shooterLeftBackspinMotorsPIDkA = new TunableDouble("/Shooter/BackSpin/Left/PID/kA",
        Constants.ShooterConstants.LeftShooterConstants.shooterBackspinMotorLeftMountedkA);

    Slot0Configs flyweelConfigLeft = new Slot0Configs();
    flyweelConfigLeft.kP = shooterLeftMainMotorsPIDkP.get();
    flyweelConfigLeft.kI = shooterLeftMainMotorsPIDkI.get();
    flyweelConfigLeft.kD = shooterLeftMainMotorsPIDkD.get();
    flyweelConfigLeft.kV = shooterLeftMainMotorsPIDkV.get();
    flyweelConfigLeft.kS = shooterLeftMainMotorsPIDkS.get();
    flyweelConfigLeft.kA = shooterLeftMainMotorsPIDkA.get();
    Slot0Configs intakeConfigLeft = new Slot0Configs();
    intakeConfigLeft.kP = shooterLeftIntakeMotorsPIDkP.get();
    intakeConfigLeft.kI = shooterLeftIntakeMotorsPIDkI.get();
    intakeConfigLeft.kD = shooterLeftIntakeMotorsPIDkD.get();
    intakeConfigLeft.kV = shooterLeftIntakeMotorsPIDkV.get();
    intakeConfigLeft.kS = shooterLeftIntakeMotorsPIDkS.get();
    intakeConfigLeft.kA = shooterLeftIntakeMotorsPIDkA.get();
    Slot0Configs backspinConfigLeft = new Slot0Configs();
    backspinConfigLeft.kP = shooterLeftBackspinMotorsPIDkP.get();
    backspinConfigLeft.kI = shooterLeftBackspinMotorsPIDkI.get();
    backspinConfigLeft.kD = shooterLeftBackspinMotorsPIDkD.get();
    backspinConfigLeft.kV = shooterLeftBackspinMotorsPIDkV.get();
    backspinConfigLeft.kS = shooterLeftBackspinMotorsPIDkS.get();
    backspinConfigLeft.kA = shooterLeftBackspinMotorsPIDkA.get();

    leftFlywheelConfig = new Configurator(flyweelConfigLeft, 0, false, true, 40);
    leftIntakeConfig = new Configurator(intakeConfigLeft, 0, false, true, 40);
    leftBackspinConfig = new Configurator(backspinConfigLeft, 0, false, true, 40);

    singleMotorShooterLeft = new ShooterModuleSingle(leftFlywheelConfig, leftBackspinConfig, leftIntakeConfig, 0);
    // right
    shooterRightMainMotorsPIDkP = new TunableDouble("/Shooter/MainMotor/Right/PID/kP",
        Constants.ShooterConstants.RightShooterConstants.kshooterMainkP);
    shooterRightMainMotorsPIDkI = new TunableDouble("/Shooter/MainMotor/Right/PID/kI",
        Constants.ShooterConstants.RightShooterConstants.kshooterMainkI);
    shooterRightMainMotorsPIDkD = new TunableDouble("/Shooter/MainMotor/Right/PID/kD",
        Constants.ShooterConstants.RightShooterConstants.kshooterMainkD);
    shooterRightMainMotorsPIDkV = new TunableDouble("/Shooter/MainMotor/Right/PID/kV",
        Constants.ShooterConstants.RightShooterConstants.kshooterMainkV);
    shooterRightMainMotorsPIDkS = new TunableDouble("/Shooter/MainMotor/Right/PID/kS",
        Constants.ShooterConstants.RightShooterConstants.kshooterMainkS);
    shooterRightMainMotorsPIDkA = new TunableDouble("/Shooter/MainMotor/Right/PID/kA",
        Constants.ShooterConstants.RightShooterConstants.kshooterMainkA);
    shooterRightIntakeMotorsPIDkP = new TunableDouble("/Shooter/Intake/Right/PID/kP",
        Constants.ShooterConstants.RightShooterConstants.shooterIntakeMotorkP);
    shooterRightIntakeMotorsPIDkI = new TunableDouble("/Shooter/Intake/Right/PID/kI",
        Constants.ShooterConstants.RightShooterConstants.shooterIntakeMotorkI);
    shooterRightIntakeMotorsPIDkD = new TunableDouble("/Shooter/Intake/Right/PID/kD",
        Constants.ShooterConstants.RightShooterConstants.shooterIntakeMotorkD);
    shooterRightIntakeMotorsPIDkV = new TunableDouble("/Shooter/Intake/Right/PID/kV",
        Constants.ShooterConstants.RightShooterConstants.shooterIntakeMotorkV);
    shooterRightIntakeMotorsPIDkS = new TunableDouble("/Shooter/Intake/Right/PID/kS",
        Constants.ShooterConstants.RightShooterConstants.shooterIntakeMotorkS);
    shooterRightIntakeMotorsPIDkA = new TunableDouble("/Shooter/Intake/Right/PID/kA",
        Constants.ShooterConstants.RightShooterConstants.shooterIntakeMotorkA);
    shooterRightBackspinMotorsPIDkP = new TunableDouble("/Shooter/BackSpin/Right/PID/kP",
        Constants.ShooterConstants.RightShooterConstants.shooterBackspinMotorRightMountedkP);
    shooterRightBackspinMotorsPIDkI = new TunableDouble("/Shooter/BackSpin/Right/PID/kI",
        Constants.ShooterConstants.RightShooterConstants.shooterBackspinMotorRightMountedkI);
    shooterRightBackspinMotorsPIDkD = new TunableDouble("/Shooter/BackSpin/Right/PID/kD",
        Constants.ShooterConstants.RightShooterConstants.shooterBackspinMotorRightMountedkD);
    shooterRightBackspinMotorsPIDkV = new TunableDouble("/Shooter/BackSpin/Right/PID/kV",
        Constants.ShooterConstants.RightShooterConstants.shooterBackspinMotorRightMountedkV);
    shooterRightBackspinMotorsPIDkS = new TunableDouble("/Shooter/BackSpin/Right/PID/kS",
        Constants.ShooterConstants.RightShooterConstants.shooterBackspinMotorRightMountedkS);
    shooterRightBackspinMotorsPIDkA = new TunableDouble("/Shooter/BackSpin/Right/PID/kA",
        Constants.ShooterConstants.RightShooterConstants.shooterBackspinMotorRightMountedkA);

    Slot0Configs flyweelConfigRight = new Slot0Configs();
    flyweelConfigRight.kP = shooterRightMainMotorsPIDkP.get();
    flyweelConfigRight.kI = shooterRightMainMotorsPIDkI.get();
    flyweelConfigRight.kD = shooterRightMainMotorsPIDkD.get();
    flyweelConfigRight.kV = shooterRightMainMotorsPIDkV.get();
    flyweelConfigRight.kS = shooterRightMainMotorsPIDkS.get();
    flyweelConfigRight.kA = shooterRightMainMotorsPIDkA.get();
    Slot0Configs intakeConfigRight = new Slot0Configs();
    intakeConfigRight.kP = shooterRightIntakeMotorsPIDkP.get();
    intakeConfigRight.kI = shooterRightIntakeMotorsPIDkI.get();
    intakeConfigRight.kD = shooterRightIntakeMotorsPIDkD.get();
    intakeConfigRight.kV = shooterRightIntakeMotorsPIDkV.get();
    intakeConfigRight.kS = shooterRightIntakeMotorsPIDkS.get();
    intakeConfigRight.kA = shooterRightIntakeMotorsPIDkA.get();
    Slot0Configs backspinConfigRight = new Slot0Configs();
    backspinConfigRight.kP = shooterRightBackspinMotorsPIDkP.get();
    backspinConfigRight.kI = shooterRightBackspinMotorsPIDkI.get();
    backspinConfigRight.kD = shooterRightBackspinMotorsPIDkD.get();
    backspinConfigRight.kV = shooterRightBackspinMotorsPIDkV.get();
    backspinConfigRight.kS = shooterRightBackspinMotorsPIDkS.get();
    backspinConfigRight.kA = shooterRightBackspinMotorsPIDkA.get();

    rightFlywheelConfig = new Configurator(flyweelConfigRight,
        Constants.ShooterConstants.RightShooterConstants.FlywheelIDRight, false, true, 40);
    rightIntakeConfig = new Configurator(intakeConfigRight,
        Constants.ShooterConstants.RightShooterConstants.intakeIDRight, false, true, 40);
    rightBackspinConfig = new Configurator(backspinConfigRight,
        Constants.ShooterConstants.RightShooterConstants.BackspinIDRight, false, true, 40);

    singleMotorShooterRight = new ShooterModuleSingle(rightFlywheelConfig, rightBackspinConfig, rightIntakeConfig, 1);

  }

  /**
   * Configures the left and right motors of the "main" flywheel these are the
   * forward bottom most motors.
   */
  public void configureShooterFlywheel() {
    Slot0Configs slot0 = new Slot0Configs();
    // right
    slot0.kP = shooterRightMainMotorsPIDkP.get();
    slot0.kI = shooterRightMainMotorsPIDkI.get();
    slot0.kD = shooterRightMainMotorsPIDkD.get();
    slot0.kV = shooterRightMainMotorsPIDkV.get();
    slot0.kS = shooterRightMainMotorsPIDkS.get();
    
    singleMotorShooterRight.configureShooterFlywheel(slot0);
    //left
    slot0.kP = shooterLeftMainMotorsPIDkP.get();
    slot0.kI = shooterLeftMainMotorsPIDkI.get();
    slot0.kD = shooterLeftMainMotorsPIDkD.get();
    slot0.kV = shooterLeftMainMotorsPIDkV.get();
    slot0.kS = shooterLeftMainMotorsPIDkS.get();


    singleMotorShooterLeft.configureShooterFlywheel(slot0);

  }

  public void configureIntakeWheel() {
    Slot0Configs slot0 = new Slot0Configs();
    // right
    slot0.kP = shooterRightIntakeMotorsPIDkP.get();
    slot0.kV = shooterRightIntakeMotorsPIDkV.get();
    slot0.kS = shooterRightIntakeMotorsPIDkS.get();
    singleMotorShooterRight.configureIntakeWheel(slot0, true);
    //left
    slot0.kP = shooterLeftIntakeMotorsPIDkP.get();
    slot0.kV = shooterLeftIntakeMotorsPIDkV.get();
    slot0.kS = shooterLeftIntakeMotorsPIDkS.get();

    singleMotorShooterLeft.configureIntakeWheel(slot0, false);

  }

  public void configurebackspinWheelMotor() {
    Slot0Configs slot0 = new Slot0Configs();
    // right
    slot0.kP = shooterRightBackspinMotorsPIDkP.get();
    slot0.kV = shooterRightBackspinMotorsPIDkV.get();
    slot0.kS = shooterRightBackspinMotorsPIDkS.get();
    singleMotorShooterRight.configurebackspinWheelMotor(slot0);
    //left
    slot0.kP = shooterLeftBackspinMotorsPIDkP.get();
    slot0.kV = shooterLeftBackspinMotorsPIDkV.get();
    slot0.kS = shooterLeftBackspinMotorsPIDkS.get();
    singleMotorShooterLeft.configurebackspinWheelMotor(slot0);
  }

  @Override
  public void updateInputs(ShooterIOInputs LeftInputs, ShooterIOInputs RightInputs) {
    singleMotorShooterLeft.updateInputs(LeftInputs);
    singleMotorShooterRight.updateInputs(RightInputs);
  }

  public void setOutput(double shooterOutput, double backspinOutput, double angleOutput) {
    singleMotorShooterLeft.setOutput(shooterOutput, backspinOutput, angleOutput);
    singleMotorShooterRight.setOutput(shooterOutput, backspinOutput, angleOutput);
  }

  public void setVelocity(ShooterState desiredState) {
    for (int i = 0; i < desiredState.getOutput().size(); i++) {
      setVelocity(desiredState, i);
    }
  }

  public void setVelocity(ShooterState desiredState, int index) {
    setVelocity(desiredState.getFlywheelSpeed(index), desiredState.getIntakeSpeed(index),
        desiredState.getBackspinSpeed(index));
  }

  public void setVelocity(double shooterFlywheelSpeed, double shooterIntakeSpeed, double shooterBackspinSpeed) {
    setMainWheelSpeed(shooterFlywheelSpeed);
    setIntakeSpeed(shooterIntakeSpeed);
    setBackspinSpeed(shooterBackspinSpeed);
  }

  public void setMainWheelSpeed(double shooterFlywheelSpeed) {
    singleMotorShooterLeft.setMainWheelSpeed(shooterFlywheelSpeed);
    singleMotorShooterRight.setMainWheelSpeed(shooterFlywheelSpeed);

  }

  public void setBackspinSpeed(double shooterBackspinSpeed) {
    singleMotorShooterLeft.setBackspinSpeed(shooterBackspinSpeed);
    singleMotorShooterRight.setBackspinSpeed(shooterBackspinSpeed);
  }

  public void setIntakeSpeed(double shooterIntakeSpeed) {
    singleMotorShooterLeft.setIntakeSpeed(shooterIntakeSpeed);
    singleMotorShooterRight.setIntakeSpeed(shooterIntakeSpeed);

  }

  @Override
  public void stop() {
    stopMainWheel();
    stopBackspinWheel();
    stopIntakeWheel();
  }

  public void stopMainWheel() {
    singleMotorShooterLeft.stopMainWheel();
    singleMotorShooterRight.stopMainWheel();

  }

  public void stopBackspinWheel() {
    singleMotorShooterLeft.stopBackspinWheel();
    singleMotorShooterRight.stopBackspinWheel();
  }

  public void stopIntakeWheel() {
    singleMotorShooterLeft.stopIntakeWheel();
    singleMotorShooterRight.stopIntakeWheel();

  }

  @Override
  public void periodic() {
    // right
    if (shooterRightMainMotorsPIDkP.hasChanged()
        || shooterRightMainMotorsPIDkI.hasChanged()
        || shooterRightMainMotorsPIDkD.hasChanged()
        || shooterRightMainMotorsPIDkS.hasChanged()
        || shooterRightMainMotorsPIDkV.hasChanged()
        || shooterRightMainMotorsPIDkA.hasChanged()) {
      configureShooterFlywheel();
    }
    if (shooterRightIntakeMotorsPIDkP.hasChanged()
        || shooterRightIntakeMotorsPIDkI.hasChanged()
        || shooterRightIntakeMotorsPIDkD.hasChanged()
        || shooterRightIntakeMotorsPIDkS.hasChanged()
        || shooterRightIntakeMotorsPIDkV.hasChanged()
        || shooterRightIntakeMotorsPIDkA.hasChanged()) {
      configureIntakeWheel();
    }
    if (shooterRightBackspinMotorsPIDkP.hasChanged()
        || shooterRightBackspinMotorsPIDkI.hasChanged()
        || shooterRightBackspinMotorsPIDkD.hasChanged()
        || shooterRightBackspinMotorsPIDkS.hasChanged()
        || shooterRightBackspinMotorsPIDkV.hasChanged()
        || shooterRightBackspinMotorsPIDkA.hasChanged()) {
      configurebackspinWheelMotor();
    }

    // left
    if (shooterLeftMainMotorsPIDkP.hasChanged()
        || shooterLeftMainMotorsPIDkI.hasChanged()
        || shooterLeftMainMotorsPIDkD.hasChanged()
        || shooterLeftMainMotorsPIDkS.hasChanged()
        || shooterLeftMainMotorsPIDkV.hasChanged()
        || shooterLeftMainMotorsPIDkA.hasChanged()) {
      configureShooterFlywheel();
    }
    if (shooterLeftIntakeMotorsPIDkP.hasChanged()
        || shooterLeftIntakeMotorsPIDkI.hasChanged()
        || shooterLeftIntakeMotorsPIDkD.hasChanged()
        || shooterLeftIntakeMotorsPIDkS.hasChanged()
        || shooterLeftIntakeMotorsPIDkV.hasChanged()
        || shooterLeftIntakeMotorsPIDkA.hasChanged()) {
      configureIntakeWheel();
    }
    if (shooterLeftBackspinMotorsPIDkP.hasChanged()
        || shooterLeftBackspinMotorsPIDkI.hasChanged()
        || shooterLeftBackspinMotorsPIDkD.hasChanged()
        || shooterLeftBackspinMotorsPIDkS.hasChanged()
        || shooterLeftBackspinMotorsPIDkV.hasChanged()
        || shooterLeftBackspinMotorsPIDkA.hasChanged()) {
      configurebackspinWheelMotor();
    }
  }

  public void simulationPeriodic() {
  }
}