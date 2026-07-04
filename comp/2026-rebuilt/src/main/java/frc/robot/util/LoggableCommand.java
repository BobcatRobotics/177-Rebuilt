package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

public class LoggableCommand {
    public static Command loggableCommand(String name, Command command) {
        return command
                .beforeStarting(() -> Logger.recordOutput("Commands/ActiveCommands/" + name, true))
                .finallyDo((interrupted) -> Logger.recordOutput("Commands/ActiveCommands/" + name,
                        false));
    }
}
