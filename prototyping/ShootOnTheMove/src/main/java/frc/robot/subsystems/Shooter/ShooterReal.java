package frc.robot.subsystems.Shooter;

import org.bobcatrobotics.Util.Tunables.TunableDouble;
import com.ctre.phoenix6.configs.Slot0Configs;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Modules.ModuleConfigurator;
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

  private ModuleConfigurator leftFlywheelConfig;
  private ModuleConfigurator leftBackspinConfig;
  private ModuleConfigurator leftIntakeConfig;
  private ModuleConfigurator rightFlywheelConfig;
  private ModuleConfigurator rightBackspinConfig;
  private ModuleConfigurator rightIntakeConfig;

  public ShooterReal() {
    // left
    shooterLeftMainMotorsPIDkP = new TunableDouble("/Shooter/Left/Flywheel/PID/kP",
        Constants.ShooterConstants.Left.kshooterMainkP);
    shooterLeftMainMotorsPIDkI = new TunableDouble("/Shooter/Left/Flywheel/PID/kI",
        Constants.ShooterConstants.Left.kshooterMainkI);
    shooterLeftMainMotorsPIDkD = new TunableDouble("/Shooter/Left/Flywheel/PID/kD",
        Constants.ShooterConstants.Left.kshooterMainkD);
    shooterLeftMainMotorsPIDkV = new TunableDouble("/Shooter/Left/Flywheel/PID/kV",
        Constants.ShooterConstants.Left.kshooterMainkV);
    shooterLeftMainMotorsPIDkS = new TunableDouble("/Shooter/Left/Flywheel/PID/kS",
        Constants.ShooterConstants.Left.kshooterMainkS);
    shooterLeftMainMotorsPIDkA = new TunableDouble("/Shooter/Left/Flywheel/PID/kA",
        Constants.ShooterConstants.Left.kshooterMainkA);
    shooterLeftIntakeMotorsPIDkP = new TunableDouble("/Shooter/Left/Intake/PID/kP",
        Constants.ShooterConstants.Left.kIntakeMotorkP);
    shooterLeftIntakeMotorsPIDkI = new TunableDouble("/Shooter/Left/Intake/PID/kI",
        Constants.ShooterConstants.Left.kIntakeMotorkI);
    shooterLeftIntakeMotorsPIDkD = new TunableDouble("/Shooter/Left/Intake/PID/kD",
        Constants.ShooterConstants.Left.kIntakeMotorkD);
    shooterLeftIntakeMotorsPIDkV = new TunableDouble("/Shooter/Left/Intake/PID/kV",
        Constants.ShooterConstants.Left.kIntakeMotorkV);
    shooterLeftIntakeMotorsPIDkS = new TunableDouble("/Shooter/Left/Intake/PID/kS",
        Constants.ShooterConstants.Left.kIntakeMotorkS);
    shooterLeftIntakeMotorsPIDkA = new TunableDouble("/Shooter/Left/Intake/PID/kA",
        Constants.ShooterConstants.Left.kIntakeMotorkA);
    shooterLeftBackspinMotorsPIDkP = new TunableDouble("/Shooter/Left/BackSpin/PID/kP",
        Constants.ShooterConstants.Left.kBackspinMotorkP);
    shooterLeftBackspinMotorsPIDkI = new TunableDouble("/Shooter/Left/BackSpin/PID/kI",
        Constants.ShooterConstants.Left.kBackspinMotorkI);
    shooterLeftBackspinMotorsPIDkD = new TunableDouble("/Shooter/Left/BackSpin/PID/kD",
        Constants.ShooterConstants.Left.kBackspinMotorkD);
    shooterLeftBackspinMotorsPIDkV = new TunableDouble("/Shooter/Left/BackSpin/PID/kV",
        Constants.ShooterConstants.Left.kBackspinMotorkV);
    shooterLeftBackspinMotorsPIDkS = new TunableDouble("/Shooter/Left/BackSpin/PID/kS",
        Constants.ShooterConstants.Left.kBackspinMotorkS);
    shooterLeftBackspinMotorsPIDkA = new TunableDouble("/Shooter/Left/BackSpin/PID/kA",
        Constants.ShooterConstants.Left.kBackspinMotorkA);

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

    leftFlywheelConfig = new ModuleConfigurator(flyweelConfigLeft, 0, false, true, 40);
    leftIntakeConfig = new ModuleConfigurator(intakeConfigLeft, 0, false, true, 40);
    leftBackspinConfig = new ModuleConfigurator(backspinConfigLeft, 0, false, true, 40);

    singleMotorShooterLeft = new ShooterModuleSingle(leftFlywheelConfig, leftBackspinConfig, leftIntakeConfig, 0);
    // right
    shooterRightMainMotorsPIDkP = new TunableDouble("/Shooter/Right/Flywheel/PID/kP",
        Constants.ShooterConstants.Right.kshooterMainkP);
    shooterRightMainMotorsPIDkI = new TunableDouble("/Shooter/Right/Flywheel/PID/kI",
        Constants.ShooterConstants.Right.kshooterMainkI);
    shooterRightMainMotorsPIDkD = new TunableDouble("/Shooter/Right/Flywheel/PID/kD",
        Constants.ShooterConstants.Right.kshooterMainkD);
    shooterRightMainMotorsPIDkV = new TunableDouble("/Shooter/Right/Flywheel/PID/kV",
        Constants.ShooterConstants.Right.kshooterMainkV);
    shooterRightMainMotorsPIDkS = new TunableDouble("/Shooter/Right/Flywheel/PID/kS",
        Constants.ShooterConstants.Right.kshooterMainkS);
    shooterRightMainMotorsPIDkA = new TunableDouble("/Shooter/Right/Flywheel/PID/kA",
        Constants.ShooterConstants.Right.kshooterMainkA);
    shooterRightIntakeMotorsPIDkP = new TunableDouble("/Shooter/Right/Intake/PID/kP",
        Constants.ShooterConstants.Right.kIntakeMotorkP);
    shooterRightIntakeMotorsPIDkI = new TunableDouble("/Shooter/Right/Intake/PID/kI",
        Constants.ShooterConstants.Right.kIntakeMotorkI);
    shooterRightIntakeMotorsPIDkD = new TunableDouble("/Shooter/Right/Intake/PID/kD",
        Constants.ShooterConstants.Right.kIntakeMotorkD);
    shooterRightIntakeMotorsPIDkV = new TunableDouble("/Shooter/Right/Intake/PID/kV",
        Constants.ShooterConstants.Right.kIntakeMotorkV);
    shooterRightIntakeMotorsPIDkS = new TunableDouble("/Shooter/Right/Intake/PID/kS",
        Constants.ShooterConstants.Right.kIntakeMotorkS);
    shooterRightIntakeMotorsPIDkA = new TunableDouble("/Shooter/Right/Intake/PID/kA",
        Constants.ShooterConstants.Right.kIntakeMotorkA);
    shooterRightBackspinMotorsPIDkP = new TunableDouble("/Shooter/Right/Backspin/PID/kP",
        Constants.ShooterConstants.Right.kBackspinMotorkP);
    shooterRightBackspinMotorsPIDkI = new TunableDouble("/Shooter/Right/Backspin/PID/kI",
        Constants.ShooterConstants.Right.kBackspinMotorkI);
    shooterRightBackspinMotorsPIDkD = new TunableDouble("/Shooter/Right/Backspin/PID/kD",
        Constants.ShooterConstants.Right.kBackspinMotorkD);
    shooterRightBackspinMotorsPIDkV = new TunableDouble("/Shooter/Right/Backspin/PID/kV",
        Constants.ShooterConstants.Right.kBackspinMotorkV);
    shooterRightBackspinMotorsPIDkS = new TunableDouble("/Shooter/Right/Backspin/PID/kS",
        Constants.ShooterConstants.Right.kBackspinMotorkS);
    shooterRightBackspinMotorsPIDkA = new TunableDouble("/Shooter/Right/Backspin/PID/kA",
        Constants.ShooterConstants.Right.kBackspinMotorkA);

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

    rightFlywheelConfig = new ModuleConfigurator(flyweelConfigRight,
        Constants.ShooterConstants.Right.FlywheelIDRight, false, true, 40);
    rightIntakeConfig = new ModuleConfigurator(intakeConfigRight,
        Constants.ShooterConstants.Right.intakeIDRight, false, true, 40);
    rightBackspinConfig = new ModuleConfigurator(backspinConfigRight,
        Constants.ShooterConstants.Right.BackspinIDRight, false, true, 40);

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
    rightFlywheelConfig = rightFlywheelConfig.apply(slot0);
    singleMotorShooterRight.configureShooterFlywheel(rightFlywheelConfig);
    //left
    slot0.kP = shooterLeftMainMotorsPIDkP.get();
    slot0.kI = shooterLeftMainMotorsPIDkI.get();
    slot0.kD = shooterLeftMainMotorsPIDkD.get();
    slot0.kV = shooterLeftMainMotorsPIDkV.get();
    slot0.kS = shooterLeftMainMotorsPIDkS.get();
    leftFlywheelConfig = leftFlywheelConfig.apply(slot0);
    singleMotorShooterLeft.configureShooterFlywheel(leftFlywheelConfig);

  }

  public void configureIntakeWheel() {
    Slot0Configs slot0 = new Slot0Configs();
    // right
    slot0.kP = shooterRightIntakeMotorsPIDkP.get();
    slot0.kI = shooterRightIntakeMotorsPIDkI.get();
    slot0.kD = shooterRightIntakeMotorsPIDkD.get();
    slot0.kV = shooterRightIntakeMotorsPIDkV.get();
    slot0.kS = shooterRightIntakeMotorsPIDkS.get();
    slot0.kA = shooterRightIntakeMotorsPIDkA.get();
    rightFlywheelConfig = rightFlywheelConfig.apply(slot0);
    singleMotorShooterRight.configureIntakeWheel(rightFlywheelConfig);
    //left
    slot0.kP = shooterLeftIntakeMotorsPIDkP.get();
    slot0.kI = shooterLeftIntakeMotorsPIDkP.get();
    slot0.kD = shooterLeftIntakeMotorsPIDkP.get();
    slot0.kV = shooterLeftIntakeMotorsPIDkP.get();
    slot0.kS = shooterLeftIntakeMotorsPIDkP.get();
    slot0.kA = shooterLeftIntakeMotorsPIDkP.get();
    leftFlywheelConfig = leftFlywheelConfig.apply(slot0);
    singleMotorShooterLeft.configureIntakeWheel(leftFlywheelConfig);

  }

  public void configurebackspinWheelMotor() {
    Slot0Configs slot0 = new Slot0Configs();
    // right
    slot0.kP = shooterRightBackspinMotorsPIDkP.get();
    slot0.kI = shooterRightBackspinMotorsPIDkP.get();
    slot0.kD = shooterRightBackspinMotorsPIDkP.get();
    slot0.kV = shooterRightBackspinMotorsPIDkP.get();
    slot0.kS = shooterRightBackspinMotorsPIDkP.get();
    slot0.kA = shooterRightBackspinMotorsPIDkP.get();
    rightFlywheelConfig = rightFlywheelConfig.apply(slot0);
    singleMotorShooterRight.configurebackspinWheelMotor(rightFlywheelConfig);
    //left
    slot0.kP = shooterLeftBackspinMotorsPIDkP.get();
    slot0.kI = shooterLeftBackspinMotorsPIDkP.get();
    slot0.kD = shooterLeftBackspinMotorsPIDkP.get();
    slot0.kV = shooterLeftBackspinMotorsPIDkP.get();
    slot0.kS = shooterLeftBackspinMotorsPIDkP.get();
    slot0.kA = shooterLeftBackspinMotorsPIDkP.get();
    leftFlywheelConfig = leftFlywheelConfig.apply(slot0);
    singleMotorShooterLeft.configurebackspinWheelMotor(leftFlywheelConfig);

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