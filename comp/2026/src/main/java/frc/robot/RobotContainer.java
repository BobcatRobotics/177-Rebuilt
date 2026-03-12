
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

import org.bobcatrobotics.Commands.ActionFactory;
import org.bobcatrobotics.Controllers.ControllerAutoDetect;
import org.bobcatrobotics.Controllers.Gamepads.ControllerBase;
import org.bobcatrobotics.GameSpecific.Rebuilt.HubData;
import org.bobcatrobotics.GameSpecific.Rebuilt.HubUtil;
import org.bobcatrobotics.Subsystems.AntiTippingLib.AntiTipping;
import org.bobcatrobotics.Subsystems.Swerve.ModuleWrapper;
import org.littletonrobotics.junction.Logger;
// import frc.robot.subsystems.roller.RollerSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAimDrive;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.hopperCharacterizationCommands;
import frc.robot.commands.shooterCharacterizationCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Hopper.HopperAutoOptions;
import frc.robot.subsystems.Hopper.HopperIO;
import frc.robot.subsystems.Hopper.HopperRealSingle;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeAutoOptions;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeReal;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.IntakeState.IntakeGoal;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterRealQuad;
import frc.robot.subsystems.Shooter.ShooterSim;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveAutoOptions;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.AllianceFlipUtil;

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
        private final Drive drive;
        private Vision vision;
        public final Shooter m_Shooter;
        private final Hopper m_Hopper;
        public final Intake intake;

        // Controller
        private final CommandXboxController controller;
        private final CommandXboxController operator;
        private final CommandXboxController devController;

        // Dashboard inputs
        private LoggedDashboardChooser<Command> autoChooser;

        private final HubUtil hub;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                controller = new CommandXboxController(0);
                operator = new CommandXboxController(1);
                devController = new CommandXboxController(2);

                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations

                                drive = new Drive(new GyroIOPigeon2(),
                                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                                new ModuleIOTalonFX(TunerConstants.BackRight));
                                // Vision
                                vision = new Vision(drive::addVisionMeasurement,
                                                new VisionIOLimelight("limelight-shooter", drive::getRotation),
                                                new VisionIOLimelight("limelight-intake", drive::getRotation));

                                m_Shooter = new Shooter(new ShooterRealQuad());
                                m_Shooter.applyState();

                                m_Hopper = new Hopper(new HopperRealSingle());
                                m_Hopper.applyState();
                                intake = new Intake(new IntakeReal());
                                intake.applyState();
                                break;
                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                drive = new Drive(new GyroIO() {
                                }, new ModuleIOSim(TunerConstants.FrontLeft),
                                                new ModuleIOSim(TunerConstants.FrontRight),
                                                new ModuleIOSim(TunerConstants.BackLeft),
                                                new ModuleIOSim(TunerConstants.BackRight));
                                m_Shooter = new Shooter(new ShooterSim());
                                m_Shooter.applyState();

                                m_Hopper = new Hopper(new HopperRealSingle());
                                m_Hopper.applyState();

                                intake = new Intake(new IntakeReal());
                                intake.applyState();

                                vision = new Vision(drive::addVisionMeasurement,
                                                new VisionIOLimelight("limelight-shooter", drive::getRotation),
                                                new VisionIOLimelight("limelight-intake", drive::getRotation));
                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                drive = new Drive(new GyroIO() {
                                }, new ModuleIO() {
                                }, new ModuleIO() {
                                }, new ModuleIO() {
                                },
                                                new ModuleIO() {
                                                });
                                m_Shooter = new Shooter(new ShooterIO() {
                                });
                                m_Shooter.applyState();
                                m_Hopper = new Hopper(new HopperIO() {

                                });
                                intake = new Intake(new IntakeIO() {
                                });

                                vision = new Vision(drive::addVisionMeasurement,
                                                new VisionIOLimelight("limelight-shooter", drive::getRotation),
                                                new VisionIOLimelight("limelight-intake", drive::getRotation));
                                break;
                }

                // Set up auto routines
                registerCommands();
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
                autoChooser = new DriveAutoOptions(autoChooser, drive).getOptions();
                autoChooser = new IntakeAutoOptions(autoChooser, intake).getOptions();

                autoChooser.addOption("Auto Test", new PathPlannerAuto("Example Auto"));

                // Configure the button bindings
                configureButtonBindings();

                hub = new HubUtil();
        }

        private void registerCommands() {
                NamedCommands.registerCommand(
                                "SpinUp", SpinUp());
                NamedCommands.registerCommand(
                                "ShootBalls", ShootBalls());
                NamedCommands.registerCommand(
                                "IntakeDown", IntakeDown());
                NamedCommands.registerCommand("IntakeUp",intake.retractAndStop());
                NamedCommands.registerCommand("IntakePosReset",new InstantCommand(()->intake.resetEncoder()));
                NamedCommands.registerCommand("RunIntakeRollers",RunIntakeRollers());
                NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> {
                                        intake.stop();
                                }, intake));
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

                // Default command, normal field-relative drive
                drive.setDefaultCommand(
                                DriveCommands.joystickDrive(
                                                drive,
                                                () -> -controller.getLeftY(),
                                                () -> -controller.getLeftX(),
                                                () -> -controller.getRightX()));

                m_Shooter.setDefaultCommand(new RunCommand(() -> {
                        ShooterState shooterState = RobotState.getInstance().getShooterState();
                        shooterState.setState(ShooterState.State.IDLE);
                        m_Shooter.setState(shooterState);
                }, m_Shooter));
                m_Hopper.setDefaultCommand(new RunCommand(() -> {
                        HopperState hopperState = RobotState.getInstance().getHopperState();
                        hopperState.setState(HopperState.State.IDLE);
                        m_Hopper.setState(hopperState);
                }, m_Hopper));
                intake.setDefaultCommand(new RunCommand(() -> intake.stop(), intake));

                controller.a().whileTrue(
                                new AutoAimDrive(
                                                drive,
                                                () -> -controller.getLeftY(),
                                                () -> -controller.getLeftX()));

                // Switch to X pattern when X button is pressed
                controller.x()
                                .onTrue(new ActionFactory().singleAction("X-Command", () -> drive.stopWithX(), drive));

                // Reset gyro to 0° when B button is pressed
                controller.b()
                                .onTrue(new ActionFactory().singleAction("ZeroGyroCommand",
                                                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(),
                                                                AllianceFlipUtil.apply(Rotation2d.kZero))),
                                                drive).ignoringDisable(true));

                /*
                 * Controls Shooting right bumper will start flywheels then after 1/4 of a
                 * second start the hopper enableing shots to fly.
                 * this should eventually be changed to look at if the shooter wheels are up to
                 * speed isntead of an time based approach.
                 */
                controller.rightBumper().whileTrue(SpinUp());
                controller.leftBumper().whileTrue(ShootBalls());
                operator.povDown().whileTrue(IntakeDown()).onFalse(new InstantCommand(() -> {
                        intake.stop();
                }, intake));
                operator.povUp().whileTrue(intake.retractAndStop());
                operator.y().onTrue(new InstantCommand(
                                () -> intake.resetEncoder()).ignoringDisable(true));

                operator.x().whileTrue(RunIntakeRollers())
                                .onFalse(new InstantCommand(() -> {
                                        intake.stop();
                                }, intake));

                double runTestTime = 5;
                Command strafeForward = DriveCommands.joystickDrive(drive, () -> 1.0, () -> 0.0, () -> 0.0)
                                .withTimeout(runTestTime)
                                .andThen(new InstantCommand(() -> drive.stop()).withTimeout(runTestTime));
                Command strafeRight = DriveCommands.joystickDrive(drive, () -> 0.0, () -> 1.0, () -> 0.0)
                                .withTimeout(runTestTime)
                                .andThen(new InstantCommand(() -> drive.stop()).withTimeout(runTestTime));
                Command strafeBackward = DriveCommands.joystickDrive(drive, () -> -1.0, () -> 0.0, () -> 0.0)
                                .withTimeout(runTestTime)
                                .andThen(new InstantCommand(() -> drive.stop()).withTimeout(runTestTime));
                Command strafeLeft = DriveCommands.joystickDrive(drive, () -> 0.0, () -> -1.0, () -> 0.0)
                                .withTimeout(runTestTime)
                                .andThen(new InstantCommand(() -> drive.stop()).withTimeout(runTestTime));
                Command rotateClockwise = DriveCommands.joystickDrive(drive, () -> 0.0, () -> 0.0, () -> 1.0)
                                .withTimeout(runTestTime)
                                .andThen(new InstantCommand(() -> drive.stop()).withTimeout(runTestTime));
                Command rotateCounterClockwise = DriveCommands.joystickDrive(drive, () -> 0.0, () -> 0.0, () -> -1.0)
                                .withTimeout(runTestTime)
                                .andThen(new InstantCommand(() -> drive.stop()).withTimeout(runTestTime));
                Command swerveCommand = strafeForward.andThen(strafeRight).andThen(strafeBackward).andThen(strafeLeft)
                                .andThen(rotateClockwise).andThen(rotateCounterClockwise);
                Command runShooterFlywheel = new RunCommand(() -> {
                        m_Shooter.shootFuel();
                }, m_Shooter).withTimeout(runTestTime).andThen(new InstantCommand(() -> m_Shooter.stop()));
                Command runHopper = new RunCommand(() -> {
                        m_Hopper.runHopper();
                }, m_Hopper).withTimeout(runTestTime).andThen(new InstantCommand(() -> m_Hopper.stop()));
                Command runIntake = new RunCommand(() -> {
                        intake.retractIntake();
                }, m_Hopper).withTimeout(runTestTime).andThen(new RunCommand(() -> {
                        intake.grabBalls();
                }, intake).withTimeout(runTestTime)).andThen(new InstantCommand(() -> intake.stop()));
                devController.leftBumper().whileTrue(
                                swerveCommand.andThen(runShooterFlywheel).andThen(runHopper).andThen(runIntake));
                devController.rightBumper().whileTrue(characterizeAll());
        }

        public Command characterizeAll() {
                Command shooterFeeder = new InstantCommand(() -> {
                        RobotState.getInstance().characterizationType = CharacterizationType.SHOOTER_FEEDER;
                }).andThen(shooterCharacterizationCommands.feedforwardCharacterization_Intake(m_Shooter))
                                .withTimeout(15).andThen(new InstantCommand(() -> m_Shooter.stopIntakeWheel()));
                Command shooterMainFlywheel = new InstantCommand(() -> {
                        RobotState.getInstance().characterizationType = CharacterizationType.SHOOTER_MAIN;
                }).andThen(shooterCharacterizationCommands.feedforwardCharacterization_Flywheel(m_Shooter))
                                .withTimeout(15).andThen(new InstantCommand(() -> m_Shooter.stopIntakeWheel()));
                Command shooterHooder = new InstantCommand(() -> {
                        RobotState.getInstance().characterizationType = CharacterizationType.SHOOTER_HOOD;
                }).andThen(shooterCharacterizationCommands.feedforwardCharacterization_Hood(m_Shooter)).withTimeout(15)
                                .andThen(new InstantCommand(() -> m_Shooter.stopIntakeWheel()));

                Command hopperMain = new InstantCommand(() -> {
                        RobotState.getInstance().characterizationType = CharacterizationType.HOPPER;
                }).andThen(hopperCharacterizationCommands.feedforwardCharacterization_Hopper(m_Hopper)).withTimeout(15)
                                .andThen(new InstantCommand(() -> m_Hopper.stop()));

                return shooterFeeder.andThen(shooterMainFlywheel).andThen(shooterHooder).andThen(hopperMain);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }

        public Pose2d getPose2D() {
                return drive.getPose();
        }

        public void teleopPeriodic() {
                HubData hubData = hub.getHubData();
                Logger.recordOutput("Hub/Status", hubData.owner);
                Logger.recordOutput("Hub/TimeRemaing", hubData.timeRemaining);
                Logger.recordOutput("Hub/HubLocation/Pose3d",
                                HubUtil.getHubCoordinates(DriverStation.getAlliance().get()));
                // Logger.recordOutput("Swerve/FrontRightEncoderOffset", )
        }

        public Command SpinUp() {
                return new RunCommand(() -> {
                        m_Shooter.spinUp();
                }, m_Shooter).alongWith(new RunCommand(() -> {
                        intake.setVelocity(125);
                }, intake));
        }

        public Command ShootBalls() {
                return new RunCommand(() -> {
                        m_Hopper.runHopper();
                }, m_Hopper).alongWith(new RunCommand(() -> {
                        m_Shooter.shootFuel();
                }, m_Shooter)).alongWith(new RunCommand(() -> {
                        intake.setVelocity(125);
                }, intake));
        }

        public Command IntakeDown(){
           return   new RunCommand(() -> {
                        intake.setPosition(11.7);
                }, intake);
        }

        public Command RunIntakeRollers(){
                return new RunCommand(() -> {
                        intake.setVelocity(400);
                }, intake);
        }
}