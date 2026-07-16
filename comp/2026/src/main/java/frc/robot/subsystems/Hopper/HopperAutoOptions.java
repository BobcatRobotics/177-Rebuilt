package frc.robot.subsystems.Hopper;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.hopperCharacterizationCommands;

public class HopperAutoOptions {
    LoggedDashboardChooser<Command> autoChooser;
    Hopper hopper;

    public HopperAutoOptions(LoggedDashboardChooser<Command> autoChooser, Hopper name) {
        this.autoChooser = autoChooser;
        this.hopper = name;
    }

    public LoggedDashboardChooser<Command> getOptions() {
        // Set up SysId routines
        autoChooser.addOption("Hopper Simple FF Characterization",
                hopperCharacterizationCommands.feedforwardCharacterization_Hopper(hopper).withTimeout(15));
        autoChooser.addOption("Hopper All Simple FF Characterization",
                hopperCharacterizationCommands.characterizeAll(hopper).withTimeout(15));

        autoChooser.addOption("Hopper Flywheel SysId (Quasistatic Forward)",
                hopper.getRegistry().get("SysIdStateHopper").quasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Hopper Flywheel SysId (Quasistatic Reverse)",
                hopper.getRegistry().get("SysIdStateHopper").quasistatic(SysIdRoutine.Direction.kReverse));

        autoChooser.addOption("Hopper Flywheel SysId (Dynamic Forward)",
                hopper.getRegistry().get("SysIdStateHopper").dynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Hopper Flywheel SysId (Dynamic Reverse)",
                hopper.getRegistry().get("SysIdStateHopper").dynamic(SysIdRoutine.Direction.kReverse));

        return autoChooser;
    }
}
