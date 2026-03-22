package frc.robot.utils.commands.ext;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemExt extends SubsystemBase {
    public Command instantRunEnd(final Runnable start, final Runnable run, final Runnable end) {
        return CommandsExt.instantRunEnd(start, run, end, this);
    }

    public Command instantRun(final Runnable start, final Runnable run) {
        return CommandsExt.instantRun(start, run, this);
    }

    public Command startIdle(final Runnable start) {
        return CommandsExt.startIdle(start, this);
    }
}
