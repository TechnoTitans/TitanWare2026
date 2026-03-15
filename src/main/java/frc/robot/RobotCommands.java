package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.slide.IntakeSlide;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.superstructure.ShotCalculator;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.commands.LoggedTrigger;
import frc.robot.utils.teleop.SwerveSpeed;

import java.util.function.Supplier;

public class RobotCommands {
    protected static final String LogKey = "RobotCommands";
    protected static final double AllowableSpeedToShootMetersPerSec = 0.1;

    private final Swerve swerve;
    private final IntakeRoller intakeRoller;
    private final IntakeSlide intakeSlide;
    private final Superstructure superstructure;
    private final Spindexer spindexer;
    private final Feeder feeder;

    private final Supplier<ShotCalculator.Target> targetSupplier;

    private final LoggedTrigger ableToShoot;

    public enum ScoringMode {
        Stationary,
        Moving
    }

    public RobotCommands(
            final Swerve swerve,
            final IntakeRoller intakeRoller,
            final IntakeSlide intakeSlide,
            final Superstructure superstructure,
            final Spindexer spindexer,
            final Feeder feeder,
            final Supplier<ShotCalculator.Target> targetSupplier
    ) {
        this.swerve = swerve;
        this.intakeRoller = intakeRoller;
        this.intakeSlide = intakeSlide;
        this.superstructure = superstructure;
        this.spindexer = spindexer;
        this.feeder = feeder;
        this.targetSupplier = targetSupplier;

        final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

        this.ableToShoot = group.t(
                "AbleToShoot",
                () -> {
                    final ChassisSpeeds swerveChassisSpeed = swerve.getRobotRelativeSpeeds();

                    return Math.hypot(
                            swerveChassisSpeed.vxMetersPerSecond,
                            swerveChassisSpeed.vyMetersPerSecond
                    ) < AllowableSpeedToShootMetersPerSec;
                }
        );
    }

    public static double linearSpeed(final ChassisSpeeds speeds) {
        return Math.hypot(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond
        );
    }

    public Command deployIntake() {
        return Commands.sequence(
                intakeSlide.setGoalCommand(IntakeSlide.Goal.EXTEND),
                Commands.waitUntil(intakeSlide.atSlideSetpoint),
                intakeRoller.setGoal(IntakeRoller.Goal.INTAKE)
        ).withName("DeployIntake");
    }

    public Command stowIntake() {
        return Commands.parallel(
                intakeSlide.setGoalCommand(IntakeSlide.Goal.STOW),
                intakeRoller.setGoal(IntakeRoller.Goal.STOP)
        ).withName("StowIntake");
    }

    public Command shootWhileMoving() {
        return Commands.parallel(
                superstructure.toGoal(Superstructure.Goal.SHOOTING),
                Commands.repeatingSequence(
                        Commands.parallel(
                                Commands.waitUntil(superstructure.atHoodSetpoint),
                                Commands.waitUntil(superstructure.atTurretSetpoint),
                                Commands.waitUntil(
                                        superstructure.atShooterSetpoint
                                                .debounce(0.1, Debouncer.DebounceType.kFalling))
                                        .withTimeout(1.5)
                        ),
                        Commands.parallel(
                                feeder.toGoal(Feeder.Goal.FEED),
                                spindexer.toGoal(Spindexer.Goal.FEED)
                        )
                                .onlyWhile(
                                        superstructure.atHoodSetpoint
                                                .and(superstructure.atTurretSetpoint)
                                                .and(superstructure.atShooterSetpoint
                                                        .debounce(0.5, Debouncer.DebounceType.kFalling))
                                )
                ),
                Commands.deferredProxy(() -> intakeSlide.toGoal(IntakeSlide.Goal.SHOOTING))
                        .unless(() -> targetSupplier.get() == ShotCalculator.Target.FERRYING),
                Commands.runOnce(() -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.SHOOTING))
                        .unless(() -> targetSupplier.get() == ShotCalculator.Target.FERRYING)
        )
            .finallyDo(() -> {
                SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.NORMAL);
                intakeSlide.setGoalCommand(IntakeSlide.Goal.EXTEND);
            })
            .withName("ShootWhileMoving");
    }

    public Command shootStationary() {
        return Commands.parallel(
                superstructure.toGoal(Superstructure.Goal.SHOOTING),
                Commands.sequence(
                        Commands.waitUntil(ableToShoot),
                        Commands.parallel(
                                Commands.repeatingSequence(
                                        Commands.waitUntil(superstructure.atSetpoint),
                                        Commands.parallel(
                                                        feeder.toGoal(Feeder.Goal.FEED),
                                                        spindexer.toGoal(Spindexer.Goal.FEED)
                                                )
                                                .onlyWhile(
                                                        superstructure.atHoodSetpoint
                                                                .and(superstructure.atTurretSetpoint)
                                                                .and(superstructure.atShooterSetpoint
                                                                        .debounce(0.5, Debouncer.DebounceType.kFalling))
                                                )
                                )
                        )
                ),
                intakeSlide.toGoal(IntakeSlide.Goal.SHOOTING)
                        .onlyIf(() -> targetSupplier.get() != ShotCalculator.Target.FERRYING),
                swerve.runWheelXCommand()
        )
                .finallyDo(() -> intakeSlide.setGoalCommand(IntakeSlide.Goal.EXTEND))
                .withName("ShootStationary");
    }

    public Command shootNoCheck() {
        return Commands.parallel(
                        superstructure.toGoal(Superstructure.Goal.SHOOTING),
                        feeder.toGoal(Feeder.Goal.FEED),
                        intakeSlide.toGoal(IntakeSlide.Goal.SHOOTING)
                                .onlyIf(() -> targetSupplier.get() != ShotCalculator.Target.FERRYING),
                        spindexer.toGoal(Spindexer.Goal.FEED),
                        Commands.runOnce(() -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.SHOOTING))
                )
                .finallyDo(() -> {
                    SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.NORMAL);
                    intakeSlide.setGoalCommand(IntakeSlide.Goal.EXTEND);
                })
                .withName("ShootWhileMoving");
    }

    public Command shootSuperstructureZero() {
        return Commands.parallel(
                        superstructure.toGoal(Superstructure.Goal.SHOOTING_STOW),
                        feeder.toGoal(Feeder.Goal.FEED),
                        intakeSlide.toGoal(IntakeSlide.Goal.SHOOTING),
                        spindexer.toGoal(Spindexer.Goal.FEED),
                        Commands.runOnce(() -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.SHOOTING))
                )
                .finallyDo(() -> {
                    SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.NORMAL);
                    intakeSlide.setGoalCommand(IntakeSlide.Goal.EXTEND);
                })
                .withName("ShootWhileMoving");
    }
}