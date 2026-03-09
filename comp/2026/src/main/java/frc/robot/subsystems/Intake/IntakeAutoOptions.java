package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.intakeCharacterizationCommands;

public class IntakeAutoOptions {
    LoggedDashboardChooser<Command> autoChooser;
    Intake intake;

    public IntakeAutoOptions(LoggedDashboardChooser<Command> autoChooser, Intake name) {
        this.autoChooser = autoChooser;
        this.intake = name;
    }

    public LoggedDashboardChooser<Command> getOptions() {
        // Set up SysId routines
        autoChooser.addOption("Intake Simple FF Characterization",
                intakeCharacterizationCommands.feedforwardCharacterization_IntakeVelocity(intake).withTimeout(15));
        autoChooser.addOption("Intake All Simple FF Characterization",
                 intakeCharacterizationCommands.characterizeForAll(intake).withTimeout(15));

        // autoChooser.addOption("Intake Flywheel SysId (Quasistatic Forward)",
        //         intake.getRegistry().get("SysIdStateRoller").quasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption("Intake Flywheel SysId (Quasistatic Reverse)",
        //         intake.getRegistry().get("SysIdStateRoller").quasistatic(SysIdRoutine.Direction.kReverse));

        // autoChooser.addOption("Intake Flywheel SysId (Dynamic Forward)",
        //         intake.getRegistry().get("SysIdStateRoller").dynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption("Intake Flywheel SysId (Dynamic Reverse)",
        //         intake.getRegistry().get("SysIdStateRoller").dynamic(SysIdRoutine.Direction.kReverse));


        // autoChooser.addOption("Intake Pivot SysId (Quasistatic Forward)",
        //         intake.getRegistry().get("SysIdStatePivot").quasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption("Intake Pivot SysId (Quasistatic Reverse)",
        //         intake.getRegistry().get("SysIdStatePivot").quasistatic(SysIdRoutine.Direction.kReverse));

        // autoChooser.addOption("Intake Pivot SysId (Dynamic Forward)",
        //         intake.getRegistry().get("SysIdStatePivot").dynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption("Intake Pivot SysId (Dynamic Reverse)",
        //         intake.getRegistry().get("SysIdStatePivot").dynamic(SysIdRoutine.Direction.kReverse));

        return autoChooser;
    }
}
