
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

import static frc.robot.subsystems.vision.VisionConstants.cameraConstants;

import org.bobcatrobotics.Commands.ActionFactory;
import org.bobcatrobotics.GameSpecific.Rebuilt.HubData;
import org.bobcatrobotics.GameSpecific.Rebuilt.HubUtil;
import org.littletonrobotics.junction.Logger;
// import frc.robot.subsystems.roller.RollerSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.carwash.Carwash;
import frc.robot.subsystems.carwash.CarwashIO;
import frc.robot.subsystems.carwash.CarwashReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.LoggableCommand;

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
        public final Carwash m_Carwash;
        private final Hopper m_Hopper;
        public final Intake intake;

        // Controller
        private final CommandXboxController controller = new CommandXboxController(0);

        // Dashboard inputs
        private final LoggedDashboardChooser<Command> autoChooser;

        private final HubUtil hub;

        Field2d field = new Field2d();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                RobotController.setBrownoutVoltage(6);
                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations
                                drive = new Drive(new GyroIOPigeon2(),
                                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                                new ModuleIOTalonFX(TunerConstants.BackRight)); // Vision
                                vision = new Vision(drive::addVisionMeasurement,
                                                new VisionIOLimelight(cameraConstants[0].name, drive::getRotation),
                                                // new VisionIOLimelight(cameraConstants[1].name, drive::getRotation),
                                                new VisionIOLimelight(cameraConstants[1].name, drive::getRotation),
                                                new VisionIOLimelight(cameraConstants[2].name, drive::getRotation),
                                                new VisionIOLimelight(cameraConstants[3].name, drive::getRotation));
                                m_Shooter = new Shooter(new ShooterReal());
                                m_Carwash = new Carwash(new CarwashReal());
                                m_Hopper = new Hopper(new HopperReal());
                                intake = new Intake(new IntakeReal());
                                break;
                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                drive = new Drive(new GyroIO() {
                                }, new ModuleIOSim(TunerConstants.FrontLeft),
                                                new ModuleIOSim(TunerConstants.FrontRight),
                                                new ModuleIOSim(TunerConstants.BackLeft),
                                                new ModuleIOSim(TunerConstants.BackRight));
                                // Vision (0 = shooter, 1 = intake, 2 = fleft, 3 = fright)
                                vision = new Vision(drive::addVisionMeasurement,
                                                new VisionIOLimelight(cameraConstants[0].name, drive::getRotation),
                                                new VisionIOLimelight(cameraConstants[1].name, drive::getRotation),
                                                new VisionIOLimelight(cameraConstants[2].name, drive::getRotation),
                                                new VisionIOLimelight(cameraConstants[3].name, drive::getRotation));
                                m_Shooter = new Shooter(new ShooterReal());
                                m_Carwash = new Carwash(new CarwashReal());
                                m_Hopper = new Hopper(new HopperReal());
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
                                // Vision (0 = shooter, 1 = intake, 2 = fleft, 3 = fright)
                                vision = new Vision(drive::addVisionMeasurement,
                                                new VisionIOLimelight(cameraConstants[0].name, drive::getRotation),
                                                new VisionIOLimelight(cameraConstants[1].name, drive::getRotation),
                                                new VisionIOLimelight(cameraConstants[2].name, drive::getRotation),
                                                new VisionIOLimelight(cameraConstants[3].name, drive::getRotation));
                                m_Shooter = new Shooter(new ShooterIO(){});
                                m_Carwash = new Carwash(new CarwashIO(){});
                                m_Hopper = new Hopper(new HopperIO(){});
                                intake = new Intake(new IntakeIO(){});
                                break;
                }

                registerNamedCammands();

                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

                // Manually input PathPlanner Autos
                autoChooser.addOption("Middle Sweep",
                                new PathPlannerAuto("Middle Sweep"));

                // Configure the button bindings
                configureButtonBindings();

                hub = new HubUtil();
        }

        /**
         * Use this method to register all named commands that will be used by this
         * application.
         */
        private void registerNamedCammands() {
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

                // Lock to 0° when A button is held
                controller
                                .a()
                                .whileTrue(
                                                DriveCommands.joystickDriveAtAngle(
                                                                drive,
                                                                () -> -controller.getLeftY(),
                                                                () -> -controller.getLeftX(),
                                                                () -> Rotation2d.kZero));

                // Switch to X pattern when X button is pressed
                controller.x()
                                .onTrue(new ActionFactory().singleAction("X-Command", () -> drive.stopWithX(), drive));

                // Reset gyro to 0° when B button is pressed
                controller.b()
                                .onTrue(new ActionFactory().singleAction("ZeroGyroCommand",
                                                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(),
                                                                Rotation2d.kZero)),
                                                drive).ignoringDisable(true));


                controller.leftTrigger().whileTrue(LoggableCommand.loggableCommand("Automated Interpolated Shooting Balls",conditionalInterpolatedShootSeq()));
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
                updateFieldTelemetry();
                if (DriverStation.getAlliance().isPresent()) {
                        RobotState.getInstance().alliance = DriverStation.getAlliance().get();
                }
                updateFieldTelemetry();
                updateHubTelemetry();
                updateRobotVelocities();
        }

        public void simTelePeriodic() {
                updateFieldTelemetry();
                if (DriverStation.getAlliance().isPresent()) {
                        RobotState.getInstance().alliance = DriverStation.getAlliance().get();
                }
                updateFieldTelemetry();
                updateHubTelemetry();
                updateRobotVelocities();
        }

        public void updateFieldTelemetry() {
                field.setRobotPose(RobotState.getInstance().robotPose);
                SmartDashboard.putData("Field", field);
        }

        public void updateHubTelemetry() {
                HubData hubData = hub.getHubData();
                Logger.recordOutput("Hub/Status", hubData.owner);
                Logger.recordOutput("Hub/TimeRemaing", hubData.timeRemaining);
                Logger.recordOutput("Hub/Alliance", RobotState.getInstance().alliance);
                Logger.recordOutput("Hub/MyHubLocation/Pose3d",
                                HubUtil.getMyHubCoordinates(RobotState.getInstance().alliance));
                Logger.recordOutput("Hub/ActiveHubLocation/Pose3d",
                                HubUtil.getActiveHubCoordinates(RobotState.getInstance().alliance));
                Logger.recordOutput("Hub/RobotDistanceToHub/Offset", drive.distanceToHub().getOffsetDistance());
                Logger.recordOutput("Hub/RobotDistanceToHub/Actual", drive.distanceToHub().getActualDistance());
                Logger.recordOutput("Hub/RobotIsAligned", drive.isAlignedToHub());


                Translation2d[] shotLine = m_Shooter.getShotLine(drive.distanceToHub().getActualDistance());
                Logger.recordOutput("Shooter/Hub/BallPath", shotLine);
                Translation2d[] shotToDepoLine = m_Shooter.getTargetShotLine(HubUtil.getDepoPassingCoordiante(RobotState.getInstance().alliance).toPose2d().getTranslation());
                Logger.recordOutput("Shooter/Depo/BallPath", shotToDepoLine);
                Translation2d[] shotToOutputLine = m_Shooter.getTargetShotLine(HubUtil.getOutpostPassingCoordinate(RobotState.getInstance().alliance).toPose2d().getTranslation());
                Logger.recordOutput("Shooter/Output/BallPath", shotToOutputLine);
        }

        public void updateRobotVelocities() {
                // Get normalized Velocity X,Y vectors
                double x = MathUtil.applyDeadband(-controller.getLeftY(), 0.1);
                double y = MathUtil.applyDeadband(-controller.getLeftX(), 0.1);
                RobotState.getInstance().vx = x * drive.getMaxLinearSpeedMetersPerSec();
                RobotState.getInstance().vy = y * drive.getMaxLinearSpeedMetersPerSec();
        }


        /**
         * This is the "new" conditional interpolated shooting sequence it takes the 2 commands and refactors them into one. 
         * If the left bumper is pressed it will automatically switch too the shoot sequence.
         */
        public Command conditionalInterpolatedShootSeq() {
                return Commands.run(() -> {
                        boolean isShooterAtSpeed = m_Shooter.atSpeed();
                        if (isShooterAtSpeed) {
                                //SHOOT!

                        } else {
                                // SPIN UP
                        }
                });
        }
}