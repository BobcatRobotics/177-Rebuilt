
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

import org.littletonrobotics.junction.Logger;
// import frc.robot.subsystems.roller.RollerSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Hopper.HopperIO;
import frc.robot.subsystems.Hopper.HopperRealDual;
import frc.robot.subsystems.Hopper.HopperRealSingle;
import frc.robot.subsystems.Hopper.HopperState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeReal;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterRealQuad;
import frc.robot.subsystems.Shooter.ShooterSim;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Shooter.ShooterState.ShooterGoal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.bobcatrobotics.Commands.ActionFactory;
import org.bobcatrobotics.Controllers.ControllerAutoDetect;
import org.bobcatrobotics.Controllers.Gamepads.ControllerBase;
import org.bobcatrobotics.GameSpecific.Rebuilt.HubData;
import org.bobcatrobotics.GameSpecific.Rebuilt.HubUtil;
import org.bobcatrobotics.Subsystems.AntiTippingLib.AntiTipping;
import org.bobcatrobotics.Subsystems.Swerve.ModuleWrapper;

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
        private final AntiTipping antiTipping;
        private Vision vision;
        private final Shooter m_Shooter;
        private final Hopper m_Hopper;
        private final Intake intake;

        // Controller
        private final ControllerBase controller;

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        private final HubUtil hub;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                controller = ControllerAutoDetect.createGamepad(0, "driver");

                ModuleWrapper newFrontRight = new ModuleWrapper("FrontRight.json", "FrontRight");
                ModuleWrapper newFrontLeft = new ModuleWrapper("FrontLeft.json", "FrontLeft");
                ModuleWrapper newBackLeft = new ModuleWrapper("BackLeft.json", "BackLeft");
                ModuleWrapper newBackRight = new ModuleWrapper("BackRight.json", "BackRight");
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations

                                drive = new Drive(new GyroIOPigeon2(),
                                                new ModuleIOTalonFX(newFrontLeft
                                                                .addModuleConstants(TunerConstants.FrontLeft)),
                                                new ModuleIOTalonFX(newFrontRight
                                                                .addModuleConstants(TunerConstants.FrontRight)),
                                                new ModuleIOTalonFX(newBackLeft
                                                                .addModuleConstants(TunerConstants.BackLeft)),
                                                new ModuleIOTalonFX(newBackRight
                                                                .addModuleConstants(TunerConstants.BackRight)));
                                // Vision
                                vision = new Vision(drive::addVisionMeasurement,
                                                new VisionIOLimelight("", drive::getRotation));

                                m_Shooter = new Shooter(new ShooterRealQuad());
                                m_Shooter.applyState();

                                m_Hopper = new Hopper(new HopperRealSingle());
                                intake = new Intake(new IntakeReal());
                                break;
                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                drive = new Drive(new GyroIO() {
                                }, new ModuleIOSim(newFrontLeft.addModuleConstants(TunerConstants.FrontLeft)),
                                                new ModuleIOSim(newFrontRight
                                                                .addModuleConstants(TunerConstants.FrontRight)),
                                                new ModuleIOSim(newBackLeft
                                                                .addModuleConstants(TunerConstants.BackLeft)),
                                                new ModuleIOSim(newBackRight
                                                                .addModuleConstants(TunerConstants.BackRight)));
                                m_Shooter = new Shooter(new ShooterSim());
                                m_Shooter.applyState();

                                m_Hopper = new Hopper(new HopperRealSingle());
                                intake = new Intake(new IntakeReal());
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
                                break;
                }

                antiTipping = new AntiTipping(() -> drive.getPitch(), () -> drive.getRoll(), 0.04, // kP
                                3.0, // tipping threshold (degrees)
                                2.5 // max correction speed (m/s)
                );

                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

                // Set up SysId routines
                autoChooser.addOption("Drive Wheel Radius Characterization",
                                DriveCommands.wheelRadiusCharacterization(drive));
                autoChooser.addOption("Drive Simple FF Characterization",
                                DriveCommands.feedforwardCharacterization(drive));
                autoChooser.addOption("Drive SysId (Quasistatic Forward)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption("Drive SysId (Quasistatic Reverse)",
                                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption("Drive SysId (Dynamic Forward)",
                                drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption("Drive SysId (Dynamic Reverse)",
                                drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

                // Configure the button bindings
                configureButtonBindings();

                hub = new HubUtil();
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

                // Lock to 0° when A button is held
                controller.getButton("A")
                                .whileTrue(
                                                DriveCommands.joystickDriveAtAngle(
                                                                drive,
                                                                () -> -controller.getLeftY(),
                                                                () -> -controller.getLeftX(),
                                                                () -> Rotation2d.kZero));

                // Switch to X pattern when X button is pressed
                controller.getButton("X")
                                .onTrue(new ActionFactory().singleAction("X-Command", () -> drive.stopWithX(), drive));

                // Reset gyro to 0° when B button is pressed
                controller.getButton("B")
                                .onTrue(new ActionFactory().singleAction("ZeroGyroCommand",
                                                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(),
                                                                Rotation2d.kZero)),
                                                drive).ignoringDisable(true));

                // Antitipping
                controller.getLeftBumper()
                                .whileTrue(new ActionFactory().continuousAction("DriveWithAntiTipping",
                                                () -> DriveCommands.joystickDriveWithAntiTipping(drive,
                                                                () -> -controller.getLeftY(),
                                                                () -> -controller.getLeftX(),
                                                                () -> -controller.getRightX(), antiTipping),
                                                () -> DriveCommands.joystickDriveWithAntiTipping(drive, () -> 0,
                                                                () -> 0, () -> 0,
                                                                antiTipping)));
                // Controls Shooting right bumper will start flywheels then after 1/4 of a
                // second start the hopper enableing shots to fly.
                // this should eventually be changed to look at if the shooter wheels are up to
                // speed isntead of an time based approach.
                controller.getRightBumper().whileTrue(new RunCommand(() -> {
                        m_Shooter.shootFuel();
                }, m_Shooter).alongWith(new WaitCommand(0.25).andThen(new RunCommand(() -> {
                        m_Hopper.runHopper();
                }, m_Hopper))));
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
                antiTipping.calculate();
                HubData hubData = hub.getHubData();
                Logger.recordOutput("Hub/Status", hubData.owner);
                Logger.recordOutput("Hub/TimeRemaing", hubData.timeRemaining);
                Logger.recordOutput("Hub/HubLocation/Pose3d",
                                HubUtil.getHubCoordinates(DriverStation.getAlliance().get()));
        }
}