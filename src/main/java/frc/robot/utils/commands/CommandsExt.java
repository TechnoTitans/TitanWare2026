package frc.robot.utils.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandsExt {
    public static void nonInterruptingSchedule(final Command command) {
        final CommandScheduler scheduler = CommandScheduler.getInstance();
        if (scheduler.isScheduled(command)) {
            return;
        }

        for (final Subsystem subsystem : command.getRequirements()) {
            if (scheduler.requiring(subsystem) != null) {
                return;
            }
        }

        scheduler.schedule(command);
    }

    public static Command defaultCommand(final Command command) {
        final CommandScheduler scheduler = CommandScheduler.getInstance();
        return Commands.runEnd(
                () -> nonInterruptingSchedule(command),
                () -> scheduler.cancel(command)
        ).withName(String.format("Schedule%s", command.getName()));
    }
}
