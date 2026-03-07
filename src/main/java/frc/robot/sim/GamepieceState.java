package frc.robot.sim;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.slide.IntakeSlide;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.utils.commands.LoggedTrigger;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class GamepieceState extends VirtualSubsystem {
    protected static final String LogKey = "GamepieceState";
    private final int BALL_CAPACITY = 52;

    private final IntakeRoller intakeRoller;
    private final IntakeSlide intakeSlide;
    private final Shooter shooter;

    private int ballCount = 0;

    private final LoggedTrigger.Group group;

    private final LoggedTrigger isIntaking;
    private final LoggedTrigger isShooting;

    public GamepieceState(final IntakeRoller intakeRoller, final IntakeSlide intakeSlide, final Shooter shooter) {
        this.intakeRoller = intakeRoller;
        this.intakeSlide = intakeSlide;
        this.shooter = shooter;

        this.group = LoggedTrigger.Group.from(LogKey);

        isIntaking = group.t("IsIntaking", () -> intakeRoller.isIntaking() && intakeSlide.atSlideSetpoint.getAsBoolean());

        isIntaking.whileTrue(
                Commands.run(() -> {
                    if (ballCount < BALL_CAPACITY) {
                        ballCount++;
                    }
                })
        );

        isShooting = group.t("IsShooting", shooter::isShooting);
        isShooting.whileTrue(
                Commands.runOnce(() -> {
                    if (ballCount > 0) {
                        ballCount--;
                    }
                })
        );
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LogKey + "/BallCount", ballCount);
    }
}
