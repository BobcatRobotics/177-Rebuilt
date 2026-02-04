
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
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
  private final Shooter m_shooter;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_shooter = new Shooter(new ShooterReal());
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_shooter = new Shooter(new ShooterIO() {
        });
        break;

      default:
        // Replayed robot, disable IO implementations
        m_shooter = new Shooter(new ShooterIO() {
        });
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
    ShooterState idleState = new ShooterState();
    ShooterState.State desired = ShooterState.State.MANUALBOTH;
    idleState.setState(desired);
    idleState.setManualSpeeds(Constants.ShooterConstants.idleFlywheelSpeedRPS,
        Constants.ShooterConstants.idleIntakeSpeedRPS, Constants.ShooterConstants.idleBackspinSpeedRPS);
    m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.setState(idleState), m_shooter));



    ShooterState desiredState = new ShooterState();
    desiredState.setState(desired);


    // CONTROL WHEELS INDIVIDUALLY
    controller.y().whileTrue(new RunCommand(() -> {
      for (int i = 0; i < desiredState.getOutput().size(); i++) {
        m_shooter.setMainWheelSpeed(desiredState.getFlywheelSpeed(i));
      }
    }, m_shooter))
        .onFalse(new InstantCommand(() -> m_shooter.stopMainWheel()));
    controller.b().whileTrue(new RunCommand(() -> {
      for (int i = 0; i < desiredState.getOutput().size(); i++) {
        m_shooter.setBackspinSpeed(desiredState.getBackspinSpeed(i));
      }
    }, m_shooter))
        .onFalse(new InstantCommand(() -> m_shooter.stopBackspinWheel()));
    controller.x().whileTrue(new RunCommand(() -> {
      for (int i = 0; i < desiredState.getOutput().size(); i++) {
        m_shooter.setIntakeSpeed(desiredState.getIntakeSpeed(i));
      }
    }, m_shooter))
        .onFalse(new InstantCommand(() -> m_shooter.stopIntakeWheel()));

    // Control wheels with intake A button will spin up the backspin and main
    // flywheels, right bumper will allow intaking.
    controller.rightBumper().whileTrue(new RunCommand(() -> {
      for (int i = 0; i < desiredState.getOutput().size(); i++) {
        m_shooter.setIntakeSpeed(desiredState.getIntakeSpeed(i));
      }
    }, m_shooter))
        .onFalse(new InstantCommand(() -> m_shooter.stopIntakeWheel()));
    controller.a()
        .whileTrue(new RunCommand(() -> {
          for (int i = 0; i < desiredState.getOutput().size(); i++) {
            m_shooter.setMainWheelSpeed(desiredState.getFlywheelSpeed(i));
            m_shooter.setBackspinSpeed(desiredState.getBackspinSpeed(i));
          }
        }, m_shooter)).onFalse(new InstantCommand(() -> {
          m_shooter.stopBackspinWheel();
          m_shooter.stopIntakeWheel();
        }));
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