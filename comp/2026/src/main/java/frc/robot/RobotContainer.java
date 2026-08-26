
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
import java.util.stream.Collectors;

import org.bobcatrobotics.Commands.ActionFactory;
import org.bobcatrobotics.GameSpecific.Rebuilt.HubData;
import org.bobcatrobotics.GameSpecific.Rebuilt.HubUtil;
import org.bobcatrobotics.Hardware.CAN.CANLogger;
import org.bobcatrobotics.Hardware.CAN.CanivoreReaderAdapter;
import org.bobcatrobotics.Hardware.CAN.RioReaderAdapter;
import org.bobcatrobotics.Util.CANDeviceDetails;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.ConsoleSource.RoboRIO;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Hopper.HopperIOReal;
import frc.robot.subsystems.Intake.Carwash.Carwash;
import frc.robot.subsystems.Intake.Carwash.CarwashIOReal;
import frc.robot.subsystems.Intake.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.IntakeIOReal;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIOReal;
// import frc.robot.subsystems.Shooter.ShooterRealQuad;
// import frc.robot.subsystems.Shooter.ShooterSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.DebouncedCommand;

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
        //public Drive drive;
        //public Vision vision;
        public  Shooter m_Shooter;
        public  Carwash m_Carwash;
        private Hopper m_Hopper;
        public  Intake intake;

        private RobotState robotState;
        // Controller
        private final CommandXboxController controller;
        private final CommandXboxController operator;
        private final CommandXboxController devController;

        // Dashboard inputs
        private LoggedDashboardChooser<Command> autoChooser;

        private LoggedDashboardChooser<Double> flywheelChooser;
        private LoggedDashboardChooser<Double> hoodChooser;
        private LoggedDashboardChooser<Double> carwashChooser;

        private final HubUtil hub;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table;

        Field2d field = new Field2d();

        private final CANLogger canLogger = new CANLogger(
                        List.of(
                                        new RioReaderAdapter(),
                                        new CanivoreReaderAdapter("CANivore")));

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                RobotInfo.getInstance().devices.putIfAbsent("rio", new ArrayList<CANDeviceDetails>());
                RobotInfo.getInstance().devices.putIfAbsent("CANivore", new ArrayList<CANDeviceDetails>());

                controller = new CommandXboxController(0);
                operator = new CommandXboxController(1);
                devController = new CommandXboxController(2);

                configureSwitchablePort();
                RobotController.setBrownoutVoltage(6);

                switch (Constants.currentMode) {
                        case REAL:
                                // Real robot, instantiate hardware IO implementations

                                //Add in all dumper modules
                                m_Shooter = new Shooter(new ShooterIOReal(Constants.ShooterConstants.Left.dumperLeftUpID, "rio"));

                                m_Carwash = new Carwash(new CarwashIOReal(Constants.CarwashConstants.SharedIntake.intakeIDLeft, "rio"));

                                m_Hopper = new Hopper(new HopperIOReal(Constants.HopperConstants.Top.hopperMotorId, "rio"));
                                
                                intake = new Intake(new IntakeIOReal(Constants.IntakeConstants.LeftRollerConstants.rollerMotorId, "rio"));
                                break;
                        case SIM:
                                // Sim robot, instantiate physics sim IO implementations
                                break;

                        default:
                                // Replayed robot, disable IO implementations
                                break;
                }

                // Set up auto routines
                registerCommands();
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
                // autoChooser = new DriveAutoOptions(autoChooser, drive).getOptions();
                // autoChooser = new IntakeAutoOptions(autoChooser, intake).getOptions();

                // autoChooser = new ShooterAutoOptions(autoChooser, m_Shooter).getOptions();

                // autoChooser.addOption("Long Trench Depot Sweep",
                // new PathPlannerAuto("Long Trench Depot Sweep"));
                // autoChooser.addOption("Long Trench Outpost Sweep",
                // new PathPlannerAuto("Long Trench Outpost Sweep"));

                // // autoChooser.addOption("Anand OP to Hub", new PathPlannerAuto("Anand OP to
                // // Hub"));
                // autoChooser.addOption("Anand Depot Side Clean Sweep",
                // new PathPlannerAuto("Anand Depot Side Clean Sweep"));
                // autoChooser.addOption("Anand OP Side Clean Sweep", new PathPlannerAuto("Anand
                // OP Side Clean Sweep"));
                // autoChooser.addOption("Trench Outpost Sweep", new PathPlannerAuto("Trench
                // Outpost Sweep"));
                // autoChooser.addOption("Trench Depot Sweep", new PathPlannerAuto("Trench Depot
                // Sweep"));
                // // autoChooser.addOption("Test Hopper", new PathPlannerAuto("TestHopper"));
                // autoChooser.addOption("Anand Depot Trench Shot", new PathPlannerAuto("Anand
                // Depot Trench Shot"));

                flywheelChooser = new LoggedDashboardChooser<>("Flywheel");
                hoodChooser = new LoggedDashboardChooser<>("Hood");
                carwashChooser = new LoggedDashboardChooser<>("Carwash");

                flywheelChooser.addDefaultOption("rps", 0.0);
                hoodChooser.addDefaultOption("rps", 0.0);
                carwashChooser.addDefaultOption("rps", 0.0);

                // Configure the button bindings
                configureButtonBindings();

                hub = new HubUtil();

                table = inst.getTable("CAN");

        }

        private void configureSwitchablePort() {
                PowerDistribution pdh = new PowerDistribution();
                if (pdh.getSwitchableChannel()) {
                        pdh.setSwitchableChannel(true);
                }
        }

        private void registerCommands() {
           //     
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
                
        }

        public void simulationButtonBindings() {

        }

        // public Command characterizeAll() {

        //         // Carwash Flywheel
        //         Command carwashFeeder = new InstantCommand(() -> {
        //                 RobotState.getInstance().characterizationType = CharacterizationType.SHOOTER_FEEDER;
        //         }).andThen(carwashCharacterizationCommands.feedforwardCharacterization_Intake(m_Carwash))
        //                         .withTimeout(15).andThen(new InstantCommand(() -> m_Carwash.stopFeedingFuel()));
        //         // Shooter Flywheels
        //         Command shooterMainFlywheel = new InstantCommand(() -> {
        //                 RobotState.getInstance().characterizationType = CharacterizationType.SHOOTER_MAIN;
        //         }).andThen(shooterCharacterizationCommands.feedforwardCharacterization_Flywheel(m_Shooter))
        //                         .withTimeout(15).andThen(new InstantCommand(() -> m_Shooter.stopMainWheel()));
        //         Command shooterHooder = new InstantCommand(() -> {
        //                 RobotState.getInstance().characterizationType = CharacterizationType.SHOOTER_HOOD;
        //         }).andThen(shooterCharacterizationCommands.feedforwardCharacterization_Hood(m_Shooter)).withTimeout(15)
        //                         .andThen(new InstantCommand(() -> m_Shooter.stopHoodWheel()));
        //         // Hopper Flywheels
        //         Command hopperMain = new InstantCommand(() -> {
        //                 RobotState.getInstance().characterizationType = CharacterizationType.HOPPER;
        //         }).andThen(hopperCharacterizationCommands.feedforwardCharacterization_Hopper(m_Hopper)).withTimeout(15)
        //                         .andThen(new InstantCommand(() -> m_Hopper.stop()));

        //         return carwashFeeder.andThen(shooterMainFlywheel).andThen(shooterHooder).andThen(hopperMain);
        // }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }

        // public Pose2d getPose2D() {
        //         return drive.getPose();
        // }

        // public void teleopPeriodic() {

        //         vision.periodic();
        //         if (DriverStation.getAlliance().isPresent()) {
        //                 RobotInfo.getInstance().alliance = DriverStation.getAlliance().get();
        //         }
        //         HubData hubData = hub.getHubData();
        //         Logger.recordOutput("Hub/Status", hubData.owner);
        //         Logger.recordOutput("Hub/TimeRemaing", hubData.timeRemaining);
        //         Logger.recordOutput("Hub/Alliance", RobotInfo.getInstance().alliance);
        //         Logger.recordOutput("Hub/MyHubLocation/Pose3d",
        //                         HubUtil.getMyHubCoordinates(RobotInfo.getInstance().alliance));
        //         Logger.recordOutput("Hub/ActiveHubLocation/Pose3d",
        //                         HubUtil.getActiveHubCoordinates(RobotInfo.getInstance().alliance));

        //         double x = MathUtil.applyDeadband(-controller.getLeftY(), 0.1);
        //         double y = MathUtil.applyDeadband(-controller.getLeftX(), 0.1);
        //         RobotInfo.getInstance().vx = x * drive.getMaxLinearSpeedMetersPerSec();
        //         RobotInfo.getInstance().vy = y * drive.getMaxLinearSpeedMetersPerSec();

        //         List<CANDeviceDetails> rioDevices = RobotInfo.getInstance().devices.get("rio");
        //         publishCanDevices("rio", rioDevices);
        //         List<CANDeviceDetails> canivoreDevices = RobotInfo.getInstance().devices.get("CANivore");
        //         publishCanDevices("CANivore", canivoreDevices);

        //         field.setRobotPose(RobotInfo.getInstance().robotPose);
        //         SmartDashboard.putData("Field", field);

        //         canLogger.periodic();

        // }

        public void publishCanDevices(String name, List<CANDeviceDetails> devices) {
                table.getEntry(name).setStringArray(devices.stream().map(Object::toString).toArray(String[]::new));
        }

        public void simTelePeriodic() {
                if (DriverStation.getAlliance().isPresent()) {
                        RobotInfo.getInstance().alliance = DriverStation.getAlliance().get();
                }
                HubData hubData = hub.getHubData();
                Logger.recordOutput("Hub/Status", hubData.owner);
                Logger.recordOutput("Hub/TimeRemaing", hubData.timeRemaining);
                Logger.recordOutput("Hub/Alliance", RobotInfo.getInstance().alliance);
                Logger.recordOutput("Hub/MyHubLocation/Pose3d",
                                HubUtil.getMyHubCoordinates(RobotInfo.getInstance().alliance));
                Logger.recordOutput("Hub/ActiveHubLocation/Pose3d",
                                HubUtil.getActiveHubCoordinates(RobotInfo.getInstance().alliance));
        }

        public static Command loggableCommand(String name, Command command) {
                return command
                                .beforeStarting(() -> Logger.recordOutput("Commands/ActiveCommands/" + name, true))
                                .finallyDo((interrupted) -> Logger.recordOutput("Commands/ActiveCommands/" + name,
                                                false));
        }
}