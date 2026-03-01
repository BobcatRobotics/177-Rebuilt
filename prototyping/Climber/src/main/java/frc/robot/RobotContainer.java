
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
import frc.robot.commands.climberCharacterizationCommands;
import frc.robot.subystems.Climber.Climber;
import frc.robot.subystems.Climber.ClimberIO;
import frc.robot.subystems.Climber.ClimberReal;
import frc.robot.subystems.Climber.ClimberState;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Climber m_Climber;

    private ClimberState.State desired = ClimberState.State.MANUAL;
  private ClimberState desiredState;


  // Controller
  private final ControllerBase controller;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    controller = ControllerAutoDetect.createGamepad(0, "operator");

    desiredState = new ClimberState();
    desiredState.setState(desired);


    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_Climber = new Climber(new ClimberReal());
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
         m_Climber = new Climber(new ClimberReal());
        break;

      default:
        // Replayed robot, disable IO implementations
        m_Climber = new Climber(new ClimberIO(){
          
        });
        break;
    }


    // Set up auto routines
    SendableChooser<Command> m_chooser = new SendableChooser<>();
        // Set up SysId routines
    m_chooser.addOption("Hopper Simple FF Characterization",
            climberCharacterizationCommands.feedforwardCharacterization_Climber(m_Climber).withTimeout(15));
    m_chooser.addOption("Simple FF Characterization",
            climberCharacterizationCommands.characterizeAll(m_Climber).withTimeout(15));




  
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
    //Ulf wanted to use a Joystick similar to last year so this is the closest i can get using xbox
    //Bringing out the climber will require a button press of the menu
    //Bringing in the climber will require operator to hold left rigger and right joystick pov down

    controller.getButton("menu").
    onTrue(
        new RunCommand
        (
          () -> m_Climber.deployClimber() , m_Climber
        )
    );


    
    
      
  

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