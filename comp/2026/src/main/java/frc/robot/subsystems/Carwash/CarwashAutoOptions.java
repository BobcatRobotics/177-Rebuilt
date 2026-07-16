package frc.robot.subsystems.Carwash;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.carwashCharacterizationCommands;

public class CarwashAutoOptions {
    LoggedDashboardChooser<Command> autoChooser;
    Carwash carwash;

    public CarwashAutoOptions(LoggedDashboardChooser<Command> autoChooser, Carwash name) {
        this.autoChooser = autoChooser;
        this.carwash = name;
    }

    public LoggedDashboardChooser<Command> getOptions() {
        // Set up SysId routines
        // Set up SysId routines
        autoChooser.addOption("Carwash Simple FF Characterization",
                carwashCharacterizationCommands.characterizeForAll(carwash).withTimeout(15));
        autoChooser.addOption("Carwash Intake Simple FF Characterization",
                carwashCharacterizationCommands.feedforwardCharacterization_Intake(carwash)
                        .withTimeout(15));

        autoChooser.addOption("Carwash Intake SysId (Quasistatic Forward)",
                carwash.getRegistry().get("SysIdStateIntake")
                        .quasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Carwash Intake SysId (Quasistatic Reverse)",
                carwash.getRegistry().get("SysIdStateIntake")
                        .quasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Carwash Intake SysId (Dynamic Forward)",
                carwash.getRegistry().get("SysIdStateIntake")
                        .dynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Carwash Intake SysId (Dynamic Reverse)",
                carwash.getRegistry().get("SysIdStateIntake")
                        .dynamic(SysIdRoutine.Direction.kReverse));
        return autoChooser;
    }
}
