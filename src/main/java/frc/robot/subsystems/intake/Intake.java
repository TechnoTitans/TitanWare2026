package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.intake.slide.IntakeSlide;
import frc.robot.utils.commands.LoggedTrigger;

public class Intake {
    protected static final String LogKey = "Intake";
    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final IntakeSlide slide;
    private final IntakeRollers rollers;

    private boolean intaking = false;
    public final LoggedTrigger isIntaking = group.t("isIntaking", () -> intaking);

    public Intake(
            final IntakeSlide slide,
            final IntakeRollers rollers
    ) {
        this.slide = slide;
        this.rollers = rollers;
    }

    private Command intaking() {
        return Commands.startEnd(() -> intaking = true, () -> intaking = false).withName("IntakingImpl");
    }

    public Command intake() {
        return Commands.parallel(
                intaking(),
                slide.toInstantGoal(IntakeSlide.Goal.INTAKE),
                rollers.toGoal(IntakeRollers.Goal.INTAKE)
        ).withName("Intaking");
    }

    public Command deploy() {
        return slide.toInstantGoal(IntakeSlide.Goal.INTAKE).withName("DeployIntake");
    }

    public Command stow() {
        return Commands.parallel(
                slide.toInstantGoal(IntakeSlide.Goal.STOW),
                rollers.toInstantGoal(IntakeRollers.Goal.OFF)
        ).withName("StowIntake");
    }

    public Command stowFeed() {
        return Commands.parallel(
                slide.toGoalHold(IntakeSlide.Goal.STOW_FEED),
                rollers.toGoal(IntakeRollers.Goal.INTAKE)
        ).withName("StowFeedIntake");
    }
}
