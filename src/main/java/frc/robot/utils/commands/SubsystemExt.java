package frc.robot.utils.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemExt extends SubsystemBase {
    public Command instantRunEnd(final Runnable start, final Runnable run, final Runnable end) {
        return new FunctionalCommand(
                () -> {
                    start.run();
                    run.run();
                },
                run,
                interrupted -> end.run(),
                () -> false,
                this
        );
    }

    public Command instantRun(final Runnable start, final Runnable run) {
        return startRun(
                () -> {
                    start.run();
                    run.run();
                },
                run
        );
    }
}
