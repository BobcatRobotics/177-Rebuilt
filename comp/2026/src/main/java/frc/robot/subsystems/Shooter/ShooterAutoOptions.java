package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.shooterCharacterizationCommands;

public class ShooterAutoOptions {
    LoggedDashboardChooser<Command> autoChooser;
    Shooter shooter;

    public ShooterAutoOptions(LoggedDashboardChooser<Command> autoChooser, Shooter name) {
        this.autoChooser = autoChooser;
        this.shooter = name;
    }

    public LoggedDashboardChooser<Command> getOptions() {
        // Set up SysId routines

        // Set up SysId routines
        autoChooser.addOption("Shooter Simple FF Characterization",
                shooterCharacterizationCommands.characterizeForAll(shooter).withTimeout(15));
        autoChooser.addOption("Flywheel Simple FF Characterization",
                shooterCharacterizationCommands.feedforwardCharacterization_Flywheel(shooter)
                        .withTimeout(15));
        autoChooser.addOption("Hood Simple FF Characterization",
                shooterCharacterizationCommands.feedforwardCharacterization_Hood(shooter)
                        .withTimeout(15));

        autoChooser.addOption("Flywheel SysId (Quasistatic Forward)",
                shooter.getRegistry().get("SysIdStateFlywheel")
                        .quasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Flywheel SysId (Quasistatic Reverse)",
                shooter.getRegistry().get("SysIdStateFlywheel")
                        .quasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Flywheel SysId (Dynamic Forward)",
                shooter.getRegistry().get("SysIdStateFlywheel")
                        .dynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Flywheel SysId (Dynamic Reverse)",
                shooter.getRegistry().get("SysIdStateFlywheel")
                        .dynamic(SysIdRoutine.Direction.kReverse));

        autoChooser.addOption("Hood SysId (Quasistatic Forward)",
                shooter.getRegistry().get("SysIdStateHood")
                        .quasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Hood SysId (Quasistatic Reverse)",
                shooter.getRegistry().get("SysIdStateHood")
                        .quasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Hood SysId (Dynamic Forward)",
                shooter.getRegistry().get("SysIdStateHood")
                        .dynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Hood SysId (Dynamic Reverse)",
                shooter.getRegistry().get("SysIdStateHood")
                        .dynamic(SysIdRoutine.Direction.kReverse));

        return autoChooser;
    }
}
