package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.feeder.Feeder;
import frc.robot.subsystems.indexer.spindexer.Spindexer;
import frc.robot.utils.commands.LoggedTrigger;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class Indexer extends VirtualSubsystem {
    protected static final String LogKey = "Indexer";
    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final Spindexer spindexer;
    private final Feeder feeder;

    private boolean feeding = false;
    public final LoggedTrigger isFeeding = group.t("isFeeding", () -> feeding);

    public Indexer(final Spindexer spindexer, final Feeder feeder) {
        this.spindexer = spindexer;
        this.feeder = feeder;
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LogKey + "/Feeding", feeding);
    }

    private Command feeding() {
        return Commands.runEnd(() -> feeding = true, () -> feeding = false).withName("FeedingImpl");
    }

    public Command toFeed() {
        return Commands.parallel(
                feeding(),
                spindexer.toGoal(Spindexer.Goal.FEED),
                feeder.toGoal(Feeder.Goal.FEED)
        ).withName("Feeding");
    }
}
