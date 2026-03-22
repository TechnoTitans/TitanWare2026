package frc.robot.utils.commands.ext;

import edu.wpi.first.wpilibj2.command.*;

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

    public static Command instantRunEnd(
            final Runnable start,
            final Runnable run,
            final Runnable end,
            final Subsystem... requirements
    ) {
        return new FunctionalCommand(
                () -> {
                    start.run();
                    run.run();
                },
                run,
                interrupted -> end.run(),
                () -> false,
                requirements
        );
    }

    public static Command instantRun(
            final Runnable start,
            final Runnable run,
            final Subsystem... requirements
    ) {
        return Commands.startRun(
                () -> {
                    start.run();
                    run.run();
                },
                run,
                requirements
        );
    }

    public static Command startIdle(final Runnable start, final Subsystem... requirements) {
        return Commands.startRun(start, () -> {}, requirements);
    }
}
