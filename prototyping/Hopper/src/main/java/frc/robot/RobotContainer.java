
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
// import frc.robot.subsystems.roller.RollerSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.hopperCharacterizationCommands;
import frc.robot.subystems.Hopper.Hopper;
import frc.robot.subystems.Hopper.HopperIO;
import frc.robot.subystems.Hopper.HopperRealDual;
import frc.robot.subystems.Hopper.HopperState;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Hopper m_Hopper;

    private HopperState.State desired = HopperState.State.MANUAL;
  private HopperState desiredState;


  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController hopperController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    desiredState = new HopperState();
    desiredState.setState(desired);


    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_Hopper = new Hopper(new HopperRealDual());
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_Hopper = new Hopper(new HopperRealDual());
        break;

      default:
        // Replayed robot, disable IO implementations
        m_Hopper = new Hopper(new HopperIO(){
          
        });
        break;
    }


    // Set up auto routines
    SendableChooser<Command> m_chooser = new SendableChooser<>();
        // Set up SysId routines
    m_chooser.addOption("Hopper Simple FF Characterization",
            hopperCharacterizationCommands.feedforwardCharacterization_Hopper(m_Hopper).withTimeout(15));

    m_chooser.addOption("Flywheel SysId (Quasistatic Forward)",
            m_Hopper.getRegistry().get("SysIdStateHopper").quasistatic(SysIdRoutine.Direction.kForward));
    m_chooser.addOption("Flywheel SysId (Quasistatic Reverse)",
            m_Hopper.getRegistry().get("SysIdStateHopper").quasistatic(SysIdRoutine.Direction.kReverse));

    m_chooser.addOption("Flywheel SysId (Dynamic Forward)",
            m_Hopper.getRegistry().get("SysIdStateHopper").dynamic(SysIdRoutine.Direction.kForward));
    m_chooser.addOption("Flywheel SysId (Dynamic Reverse)",
            m_Hopper.getRegistry().get("SysIdStateHopper").dynamic(SysIdRoutine.Direction.kReverse));

  
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", m_chooser);

    // Set up SysId routines

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    desiredState.setManualSpeeds(Constants.HopperConstants.idleHopperSpeed,
        Constants.HopperConstants.idleHopperSpeed);
    m_Hopper.setDefaultCommand(new RunCommand(() -> m_Hopper.setState(desiredState), m_Hopper));

    // CONTROL WHEELS INDIVIDUALLY
    controller.y().whileTrue(new RunCommand(() -> {
      m_Hopper.setBottomVelocity(desiredState.getHopperSpeedOfBottom());
    }, m_Hopper));
    controller.x().whileTrue(new RunCommand(() -> {
      m_Hopper.setTopVelocity(desiredState.getHopperSpeedOfTop());
    }, m_Hopper));  
    var sysIdFlywheelTests = m_Hopper.getRegistry().get("SysIdStateHopper").generateAllTests();
    hopperController.a().onTrue(sysIdFlywheelTests.get(SysIdModule.Test.QF));
    hopperController.b().onTrue(sysIdFlywheelTests.get(SysIdModule.Test.QR));
    hopperController.x().onTrue(sysIdFlywheelTests.get(SysIdModule.Test.DF));
    hopperController.y().onTrue(sysIdFlywheelTests.get(SysIdModule.Test.DR));
    hopperController.rightBumper().onTrue(hopperCharacterizationCommands.feedforwardCharacterization_Hopper(m_Hopper));

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