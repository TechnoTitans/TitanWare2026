package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.intake.slide.IntakeSlide;
import frc.robot.utils.commands.trigger.LoggedTrigger;

public class Intake {
    protected static final String LogKey = "Intake";
    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final IntakeSlide slide;
    private final IntakeRollers rollers;

    private boolean intaking = false;
    public final LoggedTrigger isIntaking = group.t("IsIntaking", () -> intaking);

    public Intake(
            final IntakeSlide slide,
            final IntakeRollers rollers
    ) {
        this.slide = slide;
        this.rollers = rollers;
    }

    public Command intake() {
        return Commands.parallel(
                Commands.startEnd(
                        () -> intaking = true,
                        () -> intaking = false
                ),
                slide.setGoal(IntakeSlide.Goal.EXTEND),
                rollers.toGoal(IntakeRollers.Goal.INTAKE)
        ).withName("Intake");
    }

    public Command deploy() {
        return slide.setGoal(IntakeSlide.Goal.EXTEND)
                .withName("DeployIntake");
    }

    public Command stow() {
        return Commands.parallel(
                slide.setGoal(IntakeSlide.Goal.STOW),
                Commands.sequence(
                        Commands.waitUntil(slide.atSetpoint),
                        rollers.setGoal(IntakeRollers.Goal.OFF)
                )
        ).withName("StowIntake");
    }

    public Command stowFeed() {
        return Commands.parallel(
                slide.toGoalHold(IntakeSlide.Goal.SHOOTING),
                rollers.toGoal(IntakeRollers.Goal.OFF)
        ).withName("StowFeedIntake");
    }
}
