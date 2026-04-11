package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.indexer.feeder.Feeder;
import frc.robot.subsystems.indexer.spindexer.Spindexer;

public class Indexer {
    protected static final String LogKey = "Indexer";

    private final Spindexer spindexer;
    private final Feeder feeder;

    private boolean indexing = false;
    public final Trigger isIndexing = new Trigger(() -> indexing);

    public Indexer(
            final Spindexer spindexer,
            final Feeder feeder
    ) {
        this.spindexer = spindexer;
        this.feeder = feeder;
    }

    public Command feed() {
        return Commands.parallel(
                Commands.startEnd(
                        () -> indexing = true,
                        () -> indexing = false
                ),
                spindexer.toGoal(Spindexer.Goal.FEED),
                feeder.toGoal(Feeder.Goal.FEED)
        ).withName("Feed");
    }

    public Command backOut() {
        return Commands.parallel(
                spindexer.toGoal(Spindexer.Goal.BACK_OUT),
                feeder.toGoal(Feeder.Goal.BACK_OUT)
        ).withName("BackOut");
    }

    public void setFeederTOFDetected(final boolean isDetected) {
        feeder.setTOFDetected(isDetected);
    }

    public boolean isFeederTOFDetected() {
        return feeder.isTOFDetected();
    }
}
