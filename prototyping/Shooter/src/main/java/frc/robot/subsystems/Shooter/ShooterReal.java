package frc.robot.subsystems.Shooter;

import org.bobcatrobotics.Util.Tunables.TunableDouble;
import com.ctre.phoenix6.configs.Slot0Configs;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Modules.ShooterModuleDual;
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

  public ShooterReal() {
    //left
    shooterLeftMainMotorsPIDkP = new TunableDouble("/Shooter/MainMotor/Left/PID/kP", Constants.ShooterConstants.kshooterMainP);
    shooterLeftMainMotorsPIDkI = new TunableDouble("/Shooter/MainMotor/Left/PID/kI", Constants.ShooterConstants.kshooterMainI);
    shooterLeftMainMotorsPIDkD = new TunableDouble("/Shooter/MainMotor/Left/PID/kD", Constants.ShooterConstants.kshooterMainD);
    shooterLeftMainMotorsPIDkV = new TunableDouble("/Shooter/MainMotor/Left/PID/kV", Constants.ShooterConstants.kshooterMainV);
    shooterLeftMainMotorsPIDkS = new TunableDouble("/Shooter/MainMotor/Left/PID/kS", Constants.ShooterConstants.kshooterMainS);
    shooterLeftMainMotorsPIDkA = new TunableDouble("/Shooter/MainMotor/Left/PID/kA", Constants.ShooterConstants.kshooterMainA);

    shooterLeftIntakeMotorsPIDkP = new TunableDouble("/Shooter/Intake/Left/PID/kP", Constants.ShooterConstants.kTopBottomP);
    shooterLeftIntakeMotorsPIDkV = new TunableDouble("/Shooter/Intake/Left/PID/kV", Constants.ShooterConstants.kTopBottomV);
    shooterLeftIntakeMotorsPIDkS = new TunableDouble("/Shooter/Intake/Left/PID/kS", Constants.ShooterConstants.kTopBottomS);

    shooterLeftBackspinMotorsPIDkP = new TunableDouble("/Shooter/BackSpin/Left/PID/kP", Constants.ShooterConstants.kTopTopP);
    shooterLeftBackspinMotorsPIDkV = new TunableDouble("/Shooter/BackSpin/Left/PID/kV", Constants.ShooterConstants.kTopTopV);
    shooterLeftBackspinMotorsPIDkS = new TunableDouble("/Shooter/BackSpin/Left/PID/kS", Constants.ShooterConstants.kTopTopS);
  
    Slot0Configs flyweelConfigLeft = new Slot0Configs();
    flyweelConfigLeft.kP = shooterLeftMainMotorsPIDkP.get();
    flyweelConfigLeft.kI = shooterLeftMainMotorsPIDkI.get();
    flyweelConfigLeft.kD = shooterLeftMainMotorsPIDkD.get();
    flyweelConfigLeft.kV = shooterLeftMainMotorsPIDkV.get();
    flyweelConfigLeft.kS = shooterLeftMainMotorsPIDkS.get();
    flyweelConfigLeft.kA = shooterLeftMainMotorsPIDkA.get();
    Slot0Configs intakeConfigLeft = new Slot0Configs();
    intakeConfigLeft.kP = shooterLeftIntakeMotorsPIDkP.get();
    intakeConfigLeft.kV = shooterLeftIntakeMotorsPIDkV.get();
    intakeConfigLeft.kS = shooterLeftIntakeMotorsPIDkS.get();
    Slot0Configs backspinConfigLeft = new Slot0Configs();
    backspinConfigLeft.kP = shooterLeftBackspinMotorsPIDkP.get();
    backspinConfigLeft.kV = shooterLeftBackspinMotorsPIDkV.get();
    backspinConfigLeft.kS = shooterLeftBackspinMotorsPIDkS.get();
    singleMotorShooterLeft = new ShooterModuleSingle(flyweelConfigLeft, intakeConfigLeft, backspinConfigLeft, Constants.ShooterConstants.FlywheelIDLeft, Constants.ShooterConstants.BackspinIDLeft, Constants.ShooterConstants.intakeIDLeft, false);
    //right
    shooterRightMainMotorsPIDkP = new TunableDouble("/Shooter/MainMotor/Right/PID/kP", Constants.ShooterConstants.kshooterMainP);
    shooterRightMainMotorsPIDkI = new TunableDouble("/Shooter/MainMotor/Right/PID/kI", Constants.ShooterConstants.kshooterMainI);
    shooterRightMainMotorsPIDkD = new TunableDouble("/Shooter/MainMotor/Right/PID/kD", Constants.ShooterConstants.kshooterMainD);
    shooterRightMainMotorsPIDkV = new TunableDouble("/Shooter/MainMotor/Right/PID/kV", Constants.ShooterConstants.kshooterMainV);
    shooterRightMainMotorsPIDkS = new TunableDouble("/Shooter/MainMotor/Right/PID/kS", Constants.ShooterConstants.kshooterMainS);
    shooterRightMainMotorsPIDkA = new TunableDouble("/Shooter/MainMotor/Right/PID/kA", Constants.ShooterConstants.kshooterMainA);

    shooterRightIntakeMotorsPIDkP = new TunableDouble("/Shooter/Intake/Right/PID/kP", Constants.ShooterConstants.kTopBottomP);
    shooterRightIntakeMotorsPIDkV = new TunableDouble("/Shooter/Intake/Right/PID/kV", Constants.ShooterConstants.kTopBottomV);
    shooterRightIntakeMotorsPIDkS = new TunableDouble("/Shooter/Intake/Right/PID/kS", Constants.ShooterConstants.kTopBottomS);

    shooterRightBackspinMotorsPIDkP = new TunableDouble("/Shooter/BackSpin/Right/PID/kP", Constants.ShooterConstants.kTopTopP);
    shooterRightBackspinMotorsPIDkV = new TunableDouble("/Shooter/BackSpin/Right/PID/kV", Constants.ShooterConstants.kTopTopV);
    shooterRightBackspinMotorsPIDkS = new TunableDouble("/Shooter/BackSpin/Right/PID/kS", Constants.ShooterConstants.kTopTopS);
    
    Slot0Configs flyweelConfigRight = new Slot0Configs();
    flyweelConfigRight.kP = shooterRightMainMotorsPIDkP.get();
    flyweelConfigRight.kI = shooterRightMainMotorsPIDkI.get();
    flyweelConfigRight.kD = shooterRightMainMotorsPIDkD.get();
    flyweelConfigRight.kV = shooterRightMainMotorsPIDkV.get();
    flyweelConfigRight.kS = shooterRightMainMotorsPIDkS.get();
    flyweelConfigRight.kA = shooterRightMainMotorsPIDkA.get();
    Slot0Configs intakeConfigRight = new Slot0Configs();
    intakeConfigRight.kP = shooterRightIntakeMotorsPIDkP.get();
    intakeConfigRight.kV = shooterRightIntakeMotorsPIDkV.get();
    intakeConfigRight.kS = shooterRightIntakeMotorsPIDkS.get();
    Slot0Configs backspinConfigRight = new Slot0Configs();
    backspinConfigRight.kP = shooterRightBackspinMotorsPIDkP.get();
    backspinConfigRight.kV = shooterRightBackspinMotorsPIDkV.get();
    backspinConfigRight.kS = shooterRightBackspinMotorsPIDkS.get();
   
    singleMotorShooterRight = new ShooterModuleSingle(flyweelConfigRight, intakeConfigRight, backspinConfigRight, Constants.ShooterConstants.FlywheelIDRight, Constants.ShooterConstants.BackspinIDRight, Constants.ShooterConstants.intakeIDRight, true);

  }

  /**
   * Configures the left and right motors of the "main" flywheel these are the
   * forward bottom most motors.
   */
  public void configureShooterFlywheel() {
    Slot0Configs slot0 = new Slot0Configs();
    //right
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
    //right
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
    //right
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
    setVelocity(desiredState.getFlywheelSpeed(), desiredState.getIntakeSpeed(), desiredState.getBackspinSpeed());
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
    //right
    if (shooterRightMainMotorsPIDkP.hasChanged()
        || shooterRightMainMotorsPIDkI.hasChanged()
        || shooterRightMainMotorsPIDkD.hasChanged()
        || shooterRightMainMotorsPIDkS.hasChanged()
        || shooterRightMainMotorsPIDkV.hasChanged()
        || shooterRightMainMotorsPIDkA.hasChanged()) {
      configureShooterFlywheel();
    }
    if (shooterRightIntakeMotorsPIDkP.hasChanged()
        || shooterRightIntakeMotorsPIDkS.hasChanged()
        || shooterRightIntakeMotorsPIDkV.hasChanged()) {
      configureIntakeWheel();
    }
    if (shooterRightBackspinMotorsPIDkP.hasChanged()
        || shooterRightBackspinMotorsPIDkS.hasChanged()
        || shooterRightBackspinMotorsPIDkV.hasChanged()) {
      configurebackspinWheelMotor();
    }

    //left
     if (shooterLeftMainMotorsPIDkP.hasChanged()
        || shooterLeftMainMotorsPIDkI.hasChanged()
        || shooterLeftMainMotorsPIDkD.hasChanged()
        || shooterLeftMainMotorsPIDkS.hasChanged()
        || shooterLeftMainMotorsPIDkV.hasChanged()
        || shooterLeftMainMotorsPIDkA.hasChanged()) {
      configureShooterFlywheel();
    }
    if (shooterLeftIntakeMotorsPIDkP.hasChanged()
        || shooterLeftIntakeMotorsPIDkS.hasChanged()
        || shooterLeftIntakeMotorsPIDkV.hasChanged()) {
      configureIntakeWheel();
    }
    if (shooterLeftBackspinMotorsPIDkP.hasChanged()
        || shooterLeftBackspinMotorsPIDkS.hasChanged()
        || shooterLeftBackspinMotorsPIDkV.hasChanged()) {
      configurebackspinWheelMotor();
    }
  }
}