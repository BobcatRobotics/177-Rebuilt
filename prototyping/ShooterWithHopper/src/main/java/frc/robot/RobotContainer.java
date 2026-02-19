
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

import org.bobcatrobotics.Hardware.Characterization.SysIdModule;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.hopperCharacterizationCommands;
import frc.robot.commands.shooterCharacterizationCommands;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Hopper.HopperIO;
import frc.robot.subsystems.Hopper.HopperRealDual;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Hopper.HopperState.HopperGoal;
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
        // Shooter
        private final Shooter m_Shooter;

        private ShooterState.State desiredShooter = ShooterState.State.MANUAL;
        private ShooterState desiredShooterState;

        // Hopper
        private final Hopper m_Hopper;

        private HopperState.State desiredHopper = HopperState.State.MANUAL;
        private HopperState desiredHopperState;

        // Controller
        private final CommandXboxController controller = new CommandXboxController(0);
        private final CommandXboxController flywheelController = new CommandXboxController(1);
        private final CommandXboxController backspinController = new CommandXboxController(2);
        private final CommandXboxController intakeController = new CommandXboxController(3);
        private final CommandXboxController hopperController = new CommandXboxController(4);

        // Dashboard inputs
        private LoggedDashboardChooser<Command> autoChooser;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                desiredShooterState = new ShooterState();
                desiredShooterState.setState(desiredShooter);

                desiredHopperState = new HopperState();
                desiredHopperState.setState(desiredHopper);

                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                m_Shooter = new Shooter(new ShooterRealQuad());
                                m_Shooter.applyState();
                                m_Hopper = new Hopper(new HopperRealDual());
                                m_Hopper.applyState();
                                break;
                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                m_Shooter = new Shooter(new ShooterSim());
                                m_Shooter.applyState();
                                m_Hopper = new Hopper(new HopperRealDual());
                                m_Hopper.applyState();
                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                m_Shooter = new Shooter(new ShooterIO() {
                                });
                                m_Shooter.applyState();
                                m_Hopper = new Hopper(new HopperIO() {

                                });
                                m_Hopper.applyState();
                                break;
                }

                // Set up auto routines
                // A chooser for autonomous commands
                SendableChooser<Command> m_chooser = new SendableChooser<>();
                // Set up SysId routines
                m_chooser.addOption("Shooter Simple FF Characterization",
                                shooterCharacterizationCommands.characterizeForAll(m_Shooter).withTimeout(15));
                m_chooser.addOption("Shooter Flywheel Simple FF Characterization",
                                shooterCharacterizationCommands.feedforwardCharacterization_Flywheel(m_Shooter)
                                                .withTimeout(15));
                m_chooser.addOption("Shooter Backspin Simple FF Characterization",
                                shooterCharacterizationCommands.feedforwardCharacterization_Backspin(m_Shooter)
                                                .withTimeout(15));
                m_chooser.addOption("Shooter Intake Simple FF Characterization",
                                shooterCharacterizationCommands.feedforwardCharacterization_Intake(m_Shooter)
                                                .withTimeout(15));

                m_chooser.addOption("Shooter Flywheel SysId (Quasistatic Forward)",
                                m_Shooter.getRegistry().get("SysIdStateFlywheel")
                                                .quasistatic(SysIdRoutine.Direction.kForward));
                m_chooser.addOption("Shooter Flywheel SysId (Quasistatic Reverse)",
                                m_Shooter.getRegistry().get("SysIdStateFlywheel")
                                                .quasistatic(SysIdRoutine.Direction.kReverse));
                m_chooser.addOption("Shooter Flywheel SysId (Dynamic Forward)",
                                m_Shooter.getRegistry().get("SysIdStateFlywheel")
                                                .dynamic(SysIdRoutine.Direction.kForward));
                m_chooser.addOption("Shooter Flywheel SysId (Dynamic Reverse)",
                                m_Shooter.getRegistry().get("SysIdStateFlywheel")
                                                .dynamic(SysIdRoutine.Direction.kReverse));

                m_chooser.addOption("Shooter Backspin SysId (Quasistatic Forward)",
                                m_Shooter.getRegistry().get("SysIdStateBackspin")
                                                .quasistatic(SysIdRoutine.Direction.kForward));
                m_chooser.addOption("Shooter Backspin SysId (Quasistatic Reverse)",
                                m_Shooter.getRegistry().get("SysIdStateBackspin")
                                                .quasistatic(SysIdRoutine.Direction.kReverse));
                m_chooser.addOption("Shooter Backspin SysId (Dynamic Forward)",
                                m_Shooter.getRegistry().get("SysIdStateBackspin")
                                                .dynamic(SysIdRoutine.Direction.kForward));
                m_chooser.addOption("Shooter Backspin SysId (Dynamic Reverse)",
                                m_Shooter.getRegistry().get("SysIdStateBackspin")
                                                .dynamic(SysIdRoutine.Direction.kReverse));

                m_chooser.addOption("Shooter Intake SysId (Quasistatic Forward)",
                                m_Shooter.getRegistry().get("SysIdStateIntake")
                                                .quasistatic(SysIdRoutine.Direction.kForward));
                m_chooser.addOption("Shooter Intake SysId (Quasistatic Reverse)",
                                m_Shooter.getRegistry().get("SysIdStateIntake")
                                                .quasistatic(SysIdRoutine.Direction.kReverse));
                m_chooser.addOption("Shooter Intake SysId (Dynamic Forward)",
                                m_Shooter.getRegistry().get("SysIdStateIntake")
                                                .dynamic(SysIdRoutine.Direction.kForward));
                m_chooser.addOption("Shooter Intake SysId (Dynamic Reverse)",
                                m_Shooter.getRegistry().get("SysIdStateIntake")
                                                .dynamic(SysIdRoutine.Direction.kReverse));

                // Set up SysId routines
                m_chooser.addOption("Hopper Simple FF Characterization",
                                hopperCharacterizationCommands.characterizeAll(m_Hopper).withTimeout(15));
                m_chooser.addOption("Hopper Simple Flywheel FF Characterization",
                                hopperCharacterizationCommands.feedforwardCharacterization_Hopper(m_Hopper)
                                                .withTimeout(15));

                m_chooser.addOption("Hopper Flywheel SysId (Quasistatic Forward)",
                                m_Hopper.getRegistry().get("SysIdStateHopper")
                                                .quasistatic(SysIdRoutine.Direction.kForward));
                m_chooser.addOption("Hopper Flywheel SysId (Quasistatic Reverse)",
                                m_Hopper.getRegistry().get("SysIdStateHopper")
                                                .quasistatic(SysIdRoutine.Direction.kReverse));

                m_chooser.addOption("Hopper Flywheel SysId (Dynamic Forward)",
                                m_Hopper.getRegistry().get("SysIdStateHopper")
                                                .dynamic(SysIdRoutine.Direction.kForward));
                m_chooser.addOption("Hopper Flywheel SysId (Dynamic Reverse)",
                                m_Hopper.getRegistry().get("SysIdStateHopper")
                                                .dynamic(SysIdRoutine.Direction.kReverse));
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", m_chooser);

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
                desiredShooterState.setManualSpeeds(Constants.ShooterConstants.idleFlywheelSpeedRPS,
                                Constants.ShooterConstants.idleIntakeSpeedRPS,
                                Constants.ShooterConstants.idleBackspinSpeedRightRPS,
                                Constants.ShooterConstants.idleBackspinSpeedLeftRPS);
                m_Shooter.setDefaultCommand(new RunCommand(() -> {
                        desiredShooterState.setState(ShooterState.State.IDLE);
                        m_Shooter.setState(desiredShooterState);
                }, m_Shooter));

                desiredHopperState.setManualSpeeds(Constants.HopperConstants.idleHopperSpeed,
                                Constants.HopperConstants.idleHopperSpeed);
                m_Hopper.setDefaultCommand(new RunCommand(() -> m_Hopper.setState(desiredHopperState), m_Hopper));

                // CONTROL WHEELS INDIVIDUALLY
                controller.y().whileTrue(new RunCommand(() -> {
                        desiredShooterState.setState(ShooterState.State.MANUAL);
                        ShooterGoal goal = new ShooterGoal();
                        goal.flywheelSpeed = desiredShooterState.getFlywheelSpeed();
                        goal.backspinSpeedLeft = 0;
                        goal.intakeSpeed = 0;
                        goal.backspinSpeedRight = 0;
                        desiredShooterState.setCurrentSetPoints(goal);
                        m_Shooter.setState(desiredShooterState);
                }, m_Shooter));

                controller.x().whileTrue(new RunCommand(() -> {
                        desiredShooterState.setState(ShooterState.State.MANUAL);
                        ShooterGoal goal = new ShooterGoal();
                        goal.flywheelSpeed = 0;
                        goal.backspinSpeedLeft = 0;
                        goal.intakeSpeed = desiredShooterState.getIntakeSpeed();
                        goal.backspinSpeedRight = 0;
                        desiredShooterState.setCurrentSetPoints(goal);
                        m_Shooter.setState(desiredShooterState);
                        m_Shooter.setState(desiredShooterState);
                }, m_Shooter));

                controller.b().whileTrue(new RunCommand(() -> {
                        desiredShooterState.setState(ShooterState.State.MANUAL);
                        ShooterGoal goal = new ShooterGoal();
                        goal.flywheelSpeed = 0;
                        goal.backspinSpeedLeft = desiredShooterState.getBackspinSpeedOfLeft();
                        goal.intakeSpeed = 0;
                        goal.backspinSpeedRight = desiredShooterState.getBackspinSpeedOfRight();
                        desiredShooterState.setCurrentSetPoints(goal);
                        m_Shooter.setState(desiredShooterState);
                }, m_Shooter));

                controller.rightBumper().whileTrue(new RunCommand(() -> {
                        desiredShooterState.setState(ShooterState.State.MANUAL);
                        ShooterGoal goal = new ShooterGoal();
                        goal.flywheelSpeed = desiredShooterState.getFlywheelSpeed();
                        goal.backspinSpeedLeft = desiredShooterState.getBackspinSpeedOfLeft();
                        goal.flywheelSpeed = desiredShooterState.getIntakeSpeed();
                        goal.backspinSpeedRight = desiredShooterState.getBackspinSpeedOfRight();
                        desiredShooterState.setCurrentSetPoints(goal);
                        m_Shooter.setState(desiredShooterState);
                }, m_Shooter));

                controller.leftBumper().whileTrue(new RunCommand(() -> {
                        desiredHopperState.setState(HopperState.State.MANUAL);
                        HopperGoal goal = new HopperGoal();
                        goal.hopperSpeedTop = desiredHopperState.getHopperSpeedOfTop();
                        goal.hopperSpeedBottom = desiredHopperState.getHopperSpeedOfBottom();
                        desiredHopperState.setCurrentSetPoints(goal);
                        m_Hopper.setState(desiredHopperState);
                }, m_Hopper));

                // tests for shooter & Hopper
                var sysIdShooterFlywheelTests = m_Shooter.getRegistry().get("SysIdStateShooterFlywheel").generateAllTests();
                flywheelController.a().onTrue(sysIdShooterFlywheelTests.get(SysIdModule.Test.QF));
                flywheelController.b().onTrue(sysIdShooterFlywheelTests.get(SysIdModule.Test.QR));
                flywheelController.x().onTrue(sysIdShooterFlywheelTests.get(SysIdModule.Test.DF));
                flywheelController.y().onTrue(sysIdShooterFlywheelTests.get(SysIdModule.Test.DR));
                flywheelController.rightBumper().onTrue(
                                shooterCharacterizationCommands.feedforwardCharacterization_Flywheel(m_Shooter));

                var sysIdShooterBackspinTests = m_Shooter.getRegistry().get("SysIdStateShooterBackspin").generateAllTests();
                backspinController.a().onTrue(sysIdShooterBackspinTests.get(SysIdModule.Test.QF));
                backspinController.b().onTrue(sysIdShooterBackspinTests.get(SysIdModule.Test.QR));
                backspinController.x().onTrue(sysIdShooterBackspinTests.get(SysIdModule.Test.DF));
                backspinController.y().onTrue(sysIdShooterBackspinTests.get(SysIdModule.Test.DR));
                backspinController.rightBumper().onTrue(
                                shooterCharacterizationCommands.feedforwardCharacterization_Backspin(m_Shooter));

                var sysIdShooterIntakeTests = m_Shooter.getRegistry().get("SysIdStateShooterIntake").generateAllTests();
                intakeController.a().onTrue(sysIdShooterIntakeTests.get(SysIdModule.Test.QF));
                intakeController.b().onTrue(sysIdShooterIntakeTests.get(SysIdModule.Test.QR));
                intakeController.x().onTrue(sysIdShooterIntakeTests.get(SysIdModule.Test.DF));
                intakeController.y().onTrue(sysIdShooterIntakeTests.get(SysIdModule.Test.DR));
                intakeController.rightBumper()
                                .onTrue(shooterCharacterizationCommands.feedforwardCharacterization_Intake(m_Shooter));

                var sysIdHopperFlywheelTests = m_Hopper.getRegistry().get("SysIdStateHopper").generateAllTests();
                hopperController.a().onTrue(sysIdHopperFlywheelTests.get(SysIdModule.Test.QF));
                hopperController.b().onTrue(sysIdHopperFlywheelTests.get(SysIdModule.Test.QR));
                hopperController.x().onTrue(sysIdHopperFlywheelTests.get(SysIdModule.Test.DF));
                hopperController.y().onTrue(sysIdHopperFlywheelTests.get(SysIdModule.Test.DR));
                hopperController.rightBumper()
                                .onTrue(hopperCharacterizationCommands.feedforwardCharacterization_Hopper(m_Hopper));

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