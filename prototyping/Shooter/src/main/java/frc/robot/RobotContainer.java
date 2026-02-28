
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

import org.bobcatrobotics.Controllers.ControllerAutoDetect;
import org.bobcatrobotics.Controllers.Gamepads.ControllerBase;
import org.bobcatrobotics.Hardware.Characterization.SysIdModule;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.shooterCharacterizationCommands;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterRealQuad;
import frc.robot.subsystems.Shooter.ShooterSim;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Shooter.ShooterState.ShooterGoal;

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
        private final Shooter m_Shooter;

        private ShooterState.State desired = ShooterState.State.MANUAL;
        private ShooterState desiredState;

        // Controller
        private final ControllerBase controller;
        private final ControllerBase flywheelController;
        private final ControllerBase backspinController;
        private final ControllerBase intakeController;

        // Dashboard inputs
        private LoggedDashboardChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                controller = ControllerAutoDetect.createGamepad(0, "driver");
                flywheelController = ControllerAutoDetect.createGamepad(1, "flywheel");
                backspinController = ControllerAutoDetect.createGamepad(2, "backspin");
                intakeController = ControllerAutoDetect.createGamepad(3, "intake");

                desiredState = new ShooterState();
                desiredState.setState(desired);

                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                m_Shooter = new Shooter(new ShooterRealQuad());
                                m_Shooter.applyState();
                                break;
                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                m_Shooter = new Shooter(new ShooterSim());
                                m_Shooter.applyState();
                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                m_Shooter = new Shooter(new ShooterIO() {
                                });
                                m_Shooter.applyState();
                                break;
                }

                // Set up auto routines
                // A chooser for autonomous commands
                SendableChooser<Command> m_chooser = new SendableChooser<>();
                // Set up SysId routines
                m_chooser.addOption("Simple FF Characterization",
                                shooterCharacterizationCommands.characterizeForAll(m_Shooter).withTimeout(15));
                m_chooser.addOption("Flywheel Simple FF Characterization",
                                shooterCharacterizationCommands.feedforwardCharacterization_Flywheel(m_Shooter)
                                                .withTimeout(15));
                m_chooser.addOption("Backspin Simple FF Characterization",
                                shooterCharacterizationCommands.feedforwardCharacterization_Backspin(m_Shooter)
                                                .withTimeout(15));
                m_chooser.addOption("Intake Simple FF Characterization",
                                shooterCharacterizationCommands.feedforwardCharacterization_Intake(m_Shooter)
                                                .withTimeout(15));

                m_chooser.addOption("Flywheel SysId (Quasistatic Forward)",
                                m_Shooter.getRegistry().get("SysIdStateFlywheel")
                                                .quasistatic(SysIdRoutine.Direction.kForward));
                m_chooser.addOption("Flywheel SysId (Quasistatic Reverse)",
                                m_Shooter.getRegistry().get("SysIdStateFlywheel")
                                                .quasistatic(SysIdRoutine.Direction.kReverse));
                m_chooser.addOption("Flywheel SysId (Dynamic Forward)",
                                m_Shooter.getRegistry().get("SysIdStateFlywheel")
                                                .dynamic(SysIdRoutine.Direction.kForward));
                m_chooser.addOption("Flywheel SysId (Dynamic Reverse)",
                                m_Shooter.getRegistry().get("SysIdStateFlywheel")
                                                .dynamic(SysIdRoutine.Direction.kReverse));

                m_chooser.addOption("Backspin SysId (Quasistatic Forward)",
                                m_Shooter.getRegistry().get("SysIdStateBackspin")
                                                .quasistatic(SysIdRoutine.Direction.kForward));
                m_chooser.addOption("Backspin SysId (Quasistatic Reverse)",
                                m_Shooter.getRegistry().get("SysIdStateBackspin")
                                                .quasistatic(SysIdRoutine.Direction.kReverse));
                m_chooser.addOption("Backspin SysId (Dynamic Forward)",
                                m_Shooter.getRegistry().get("SysIdStateBackspin")
                                                .dynamic(SysIdRoutine.Direction.kForward));
                m_chooser.addOption("Backspin SysId (Dynamic Reverse)",
                                m_Shooter.getRegistry().get("SysIdStateBackspin")
                                                .dynamic(SysIdRoutine.Direction.kReverse));

                m_chooser.addOption("Intake SysId (Quasistatic Forward)",
                                m_Shooter.getRegistry().get("SysIdStateIntake")
                                                .quasistatic(SysIdRoutine.Direction.kForward));
                m_chooser.addOption("Intake SysId (Quasistatic Reverse)",
                                m_Shooter.getRegistry().get("SysIdStateIntake")
                                                .quasistatic(SysIdRoutine.Direction.kReverse));
                m_chooser.addOption("Intake SysId (Dynamic Forward)",
                                m_Shooter.getRegistry().get("SysIdStateIntake")
                                                .dynamic(SysIdRoutine.Direction.kForward));
                m_chooser.addOption("Intake SysId (Dynamic Reverse)",
                                m_Shooter.getRegistry().get("SysIdStateIntake")
                                                .dynamic(SysIdRoutine.Direction.kReverse));
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", m_chooser);

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
                desiredState.setManualSpeeds(Constants.ShooterConstants.idleFlywheelSpeedRPS,
                                Constants.ShooterConstants.idleIntakeSpeedRPS,
                                Constants.ShooterConstants.idleHoodSpeedRPS);
                m_Shooter.setDefaultCommand(new RunCommand(() -> {
                        desiredState.setState(ShooterState.State.IDLE);
                        m_Shooter.setState(desiredState);
                }, m_Shooter));

                // CONTROL WHEELS INDIVIDUALLY
                controller.getButton("Y").whileTrue(new RunCommand(() -> {
                        desiredState.setState(ShooterState.State.MANUAL);
                        ShooterGoal goal = new ShooterGoal();
                        goal.flywheelSpeed = desiredState.getFlywheelSpeed();
                        goal.hoodSpeed = 0;
                        goal.intakeSpeed = 0;
                        desiredState.setCurrentSetPoints(goal);
                        m_Shooter.setState(desiredState);
                }, m_Shooter));

                controller.getButton("X").whileTrue(new RunCommand(() -> {
                        desiredState.setState(ShooterState.State.MANUAL);
                        ShooterGoal goal = new ShooterGoal();
                        goal.flywheelSpeed = 0;
                        goal.hoodSpeed = 0;
                        goal.intakeSpeed = desiredState.getIntakeSpeed();
                        desiredState.setCurrentSetPoints(goal);
                        m_Shooter.setState(desiredState);
                        m_Shooter.setState(desiredState);
                }, m_Shooter));

                controller.getButton("B").whileTrue(new RunCommand(() -> {
                        desiredState.setState(ShooterState.State.MANUAL);
                        ShooterGoal goal = new ShooterGoal();
                        goal.flywheelSpeed = 0;
                        goal.hoodSpeed = desiredState.getHoodSpeed();
                        goal.intakeSpeed = 0;
                        desiredState.setCurrentSetPoints(goal);
                        m_Shooter.setState(desiredState);
                }, m_Shooter));

                controller.getRightBumper().whileTrue(new RunCommand(() -> {
                        desiredState.setState(ShooterState.State.MANUAL);
                        ShooterGoal goal = new ShooterGoal();
                        goal.flywheelSpeed = desiredState.getFlywheelSpeed();
                        goal.hoodSpeed = desiredState.getHoodSpeed();
                        goal.intakeSpeed = desiredState.getIntakeSpeed();
                        desiredState.setCurrentSetPoints(goal);
                        m_Shooter.setState(desiredState);
                }, m_Shooter));

                var sysIdFlywheelTests = m_Shooter.getRegistry().get("SysIdStateFlywheel").generateAllTests();
                flywheelController.getButton("A").onTrue(sysIdFlywheelTests.get(SysIdModule.Test.QF));
                flywheelController.getButton("B").onTrue(sysIdFlywheelTests.get(SysIdModule.Test.QR));
                flywheelController.getButton("X").onTrue(sysIdFlywheelTests.get(SysIdModule.Test.DF));
                flywheelController.getButton("Y").onTrue(sysIdFlywheelTests.get(SysIdModule.Test.DR));
                flywheelController.getRightBumper().onTrue(
                                shooterCharacterizationCommands.feedforwardCharacterization_Flywheel(m_Shooter));

                var sysIdBackspinTests = m_Shooter.getRegistry().get("SysIdStateBackspin").generateAllTests();
                backspinController.getButton("A").onTrue(sysIdBackspinTests.get(SysIdModule.Test.QF));
                backspinController.getButton("B").onTrue(sysIdBackspinTests.get(SysIdModule.Test.QR));
                backspinController.getButton("X").onTrue(sysIdBackspinTests.get(SysIdModule.Test.DF));
                backspinController.getButton("Y").onTrue(sysIdBackspinTests.get(SysIdModule.Test.DR));
                backspinController.getRightBumper().onTrue(
                                shooterCharacterizationCommands.feedforwardCharacterization_Backspin(m_Shooter));

                var sysIdIntakeTests = m_Shooter.getRegistry().get("SysIdStateIntake").generateAllTests();
                intakeController.getButton("A").onTrue(sysIdIntakeTests.get(SysIdModule.Test.QF));
                intakeController.getButton("B").onTrue(sysIdIntakeTests.get(SysIdModule.Test.QR));
                intakeController.getButton("X").onTrue(sysIdIntakeTests.get(SysIdModule.Test.DF));
                intakeController.getButton("Y").onTrue(sysIdIntakeTests.get(SysIdModule.Test.DR));
                intakeController.getRightBumper()
                                .onTrue(shooterCharacterizationCommands.feedforwardCharacterization_Intake(m_Shooter));
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