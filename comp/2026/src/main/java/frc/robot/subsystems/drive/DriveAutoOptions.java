package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.commands.driveCharacterizationCommands;

public class DriveAutoOptions {
    LoggedDashboardChooser<Command> autoChooser;
    Drive drive;
    
    public DriveAutoOptions(LoggedDashboardChooser<Command> autoChooser, Drive name){
        this.autoChooser = autoChooser;
        this.drive = name;
    }

    public LoggedDashboardChooser<Command> getOptions(){
            // Set up SysId routines
        autoChooser.addOption("Swerve Drive Simple FF Characterization",
                        new InstantCommand(()->{RobotState.getInstance().isSimpleFFSteerMode = false;}).andThen(driveCharacterizationCommands.feedforwardCharacterization(drive)));
        autoChooser.addOption("Swerve Steer Simple FF Characterization",
                        new InstantCommand(()->{RobotState.getInstance().isSimpleFFSteerMode = true;}).andThen(driveCharacterizationCommands.feedforwardCharacterization(drive)));
        autoChooser.addOption("Swerve Wheel Radius Characterization",
                        driveCharacterizationCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Swerve SysId (Quasistatic Forward)",
                        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Swerve SysId (Quasistatic Reverse)",
                        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Swerve SysId (Dynamic Forward)",
                        drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Swerve SysId (Dynamic Reverse)",
                        drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        return autoChooser;
    }
}
