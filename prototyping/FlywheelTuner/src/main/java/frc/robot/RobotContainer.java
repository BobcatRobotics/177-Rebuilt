
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

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter.FlywheelTunerSubsystem;
import frc.robot.subsystems.Shooter.FlywheelTuningCommand;

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
        private final FlywheelTunerSubsystem shooterMainFlywheel;
        private final FlywheelTunerSubsystem shooterHood;
        // Controller
        private final CommandXboxController controller;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                controller = new CommandXboxController(0);
                List<Integer> ids = new ArrayList();
                ids.add(Constants.ShooterConstants.SharedFlywheel.FlywheelOuterIDLeft);
                ids.add(Constants.ShooterConstants.SharedFlywheel.FlywheelOuterIDRight);
                ids.add(Constants.ShooterConstants.SharedFlywheel.FlywheelInnerIDLeft);
                ids.add(Constants.ShooterConstants.SharedFlywheel.FlywheelInnerIDRight);
                List<Boolean> inversions = new ArrayList();
                inversions.add(Constants.ShooterConstants.SharedFlywheel.isInvertedOuterLeft);
                inversions.add(Constants.ShooterConstants.SharedFlywheel.isInvertedOuterRight);
                inversions.add(Constants.ShooterConstants.SharedFlywheel.isInvertedInnerLeft);
                inversions.add(Constants.ShooterConstants.SharedFlywheel.isInvertedInnerRight);
                List<Boolean> coastModes = new ArrayList();
                inversions.add(Constants.ShooterConstants.SharedFlywheel.isCoastLeft);
                inversions.add(Constants.ShooterConstants.SharedFlywheel.isCoastRight);
                inversions.add(Constants.ShooterConstants.SharedFlywheel.isCoastLeft);
                inversions.add(Constants.ShooterConstants.SharedFlywheel.isCoastRight);
                shooterMainFlywheel = new FlywheelTunerSubsystem("Main", ids, inversions, coastModes,
                                Constants.ShooterConstants.SharedFlywheel.supplyCurrentLimit,
                                Constants.ShooterConstants.SharedFlywheel.statorCurrentLimit,
                                Constants.ShooterConstants.SharedFlywheel.supplyCurrentLimit,
                                -Constants.ShooterConstants.SharedFlywheel.supplyCurrentLimit);
                ids = new ArrayList();
                ids.add(Constants.ShooterConstants.Left.HoodID);
                ids.add(Constants.ShooterConstants.Right.HoodID);
                inversions = new ArrayList();
                inversions.add(Constants.ShooterConstants.Left.isInverted);
                inversions.add(Constants.ShooterConstants.Right.isInverted);
                coastModes = new ArrayList();
                inversions.add(Constants.ShooterConstants.Left.isCoast);
                inversions.add(Constants.ShooterConstants.Right.isCoast);

                shooterHood = new FlywheelTunerSubsystem("Hood", ids, inversions, coastModes,
                                Constants.ShooterConstants.Left.supplyCurrentLimit,
                                Constants.ShooterConstants.Left.statorCurrentLimit,
                                Constants.ShooterConstants.Left.supplyCurrentLimit,
                                -Constants.ShooterConstants.Left.supplyCurrentLimit);

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
                if (Robot.isSimulation()) {

                        simulationButtonBindings();
                }

                controller.leftBumper().whileTrue(new FlywheelTuningCommand(shooterMainFlywheel, 5800))
                                .onFalse(new InstantCommand(() -> shooterMainFlywheel.stop()));
                controller.rightBumper().whileTrue(new FlywheelTuningCommand(shooterHood, 5800))
                                .onFalse(new InstantCommand(() -> shooterMainFlywheel.stop()));
        }

        public void simulationButtonBindings() {

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return new InstantCommand();
        }

        public void teleopPeriodic() {

        }

        public void simTelePeriodic() {
        }
}