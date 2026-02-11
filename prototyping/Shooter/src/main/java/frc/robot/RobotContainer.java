
// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;
// import frc.robot.subsystems.roller.RollerSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIntake;
import frc.robot.subsystems.Shooter.ShooterReal;
import frc.robot.subsystems.Shooter.ShooterState;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Shooter m_lefShooter;
  private final Shooter m_rightShooter;
  private final Shooter m_shooterIntake;

  private ShooterState.State desired = ShooterState.State.MANUAL;
  private ShooterState desiredStateLeft;
  private ShooterState desiredStateRight;
  private ShooterState desiredStateIntake;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    desiredStateLeft = new ShooterState("Left");
    desiredStateLeft.setState(desired);
    desiredStateRight = new ShooterState("Right");
    desiredStateLeft.setState(desired);
    desiredStateIntake = new ShooterState("Intake");
    desiredStateLeft.setState(desired);


    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_lefShooter = new Shooter(new ShooterReal("Left"));
        m_lefShooter.applyState();
        m_rightShooter = new Shooter(new ShooterReal("Right"));
        m_rightShooter.applyState();
        m_shooterIntake = new Shooter(new ShooterIntake("Intake"));
        m_shooterIntake.applyState();
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_lefShooter = new Shooter(new ShooterReal("Left"));
        m_lefShooter.applyState();
        m_rightShooter = new Shooter(new ShooterReal("Right"));
        m_rightShooter.applyState();
        m_shooterIntake = new Shooter(new ShooterIntake("Intake"));
        m_shooterIntake.applyState();
        break;

      default:
        // Replayed robot, disable IO implementations
        m_lefShooter = new Shooter(new ShooterIO() {
        });
        m_lefShooter.applyState();
        m_rightShooter = new Shooter(new ShooterIO() {
        });
        m_rightShooter.applyState();
        m_shooterIntake = new Shooter(new ShooterIO() {
        });
        m_shooterIntake.applyState();
        break;
    }

    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices",null);

    // Set up SysId routines

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    desiredStateLeft.setManualSpeeds(Constants.ShooterConstants.idleFlywheelSpeedRPS,
        Constants.ShooterConstants.idleIntakeSpeedRPS, Constants.ShooterConstants.idleBackspinSpeedRPS);
    m_lefShooter.setDefaultCommand(new RunCommand(() -> m_lefShooter.setState(desiredStateLeft), m_lefShooter));

    desiredStateRight.setManualSpeeds(Constants.ShooterConstants.idleFlywheelSpeedRPS,
        Constants.ShooterConstants.idleIntakeSpeedRPS, Constants.ShooterConstants.idleBackspinSpeedRPS);
    m_rightShooter.setDefaultCommand(new RunCommand(() -> m_rightShooter.setState(desiredStateRight), m_rightShooter));

    desiredStateIntake.setManualSpeeds(Constants.ShooterConstants.idleFlywheelSpeedRPS,
        Constants.ShooterConstants.idleIntakeSpeedRPS, Constants.ShooterConstants.idleBackspinSpeedRPS);
    m_shooterIntake.setDefaultCommand(new RunCommand(() -> m_shooterIntake.setState(desiredStateIntake), m_shooterIntake));


    // CONTROL WHEELS INDIVIDUALLY
    controller.y().whileTrue(new RunCommand(() -> {
      m_lefShooter.setMainWheelSpeed(desiredStateLeft.getFlywheelSpeed());
    }, m_lefShooter).alongWith(new RunCommand(() -> {
      m_rightShooter.setMainWheelSpeed(desiredStateRight.getFlywheelSpeed());
    }, m_rightShooter)));
    controller.x().whileTrue(new RunCommand(() -> {
      m_lefShooter.setBackspinSpeed(desiredStateLeft.getBackspinSpeed());
    }, m_lefShooter).alongWith(new RunCommand(() -> {
      m_rightShooter.setBackspinSpeed(desiredStateRight.getBackspinSpeed());
    }, m_rightShooter)));
    controller.a().whileTrue(new RunCommand(() -> {
      m_lefShooter.setIntakeSpeed(desiredStateIntake.getIntakeSpeed());
    }, m_lefShooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void teleopPeriodic() {

  }
}