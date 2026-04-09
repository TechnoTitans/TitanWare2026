package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.params.MovingTOFShot;
import frc.robot.subsystems.superstructure.params.ShotParameters;
import frc.robot.subsystems.superstructure.params.ShotProvider;
import frc.robot.subsystems.superstructure.params.StaticShot;
import frc.robot.utils.commands.trigger.LoggedTrigger;
import frc.robot.utils.subsystems.VirtualSubsystem;
import frc.robot.utils.teleop.SwerveSpeed;
import org.littletonrobotics.junction.Logger;

import java.util.Map;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class ShootCommands extends VirtualSubsystem {
    protected static final String LogKey = "ShootCommands";
    private static final double SwerveSpeedTolerance = 0.25;

    public enum Target {
        HUB,
        FERRY,
        FERRY_BLOCKED
    }

    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final Swerve swerve;
    private final Intake intake;
    private final Indexer indexer;
    private final FuelState fuelState;
    private final Superstructure superstructure;

    private final Supplier<Target> targetSupplier;
    private final Supplier<Pose2d> targetPoseSupplier;

    private final ShotProvider<ShotProvider.Kind.Static> staticShotProvider;
    private final Supplier<ShotParameters> staticShot;

    private final ShotProvider<ShotProvider.Kind.Moving> movingShotProvider;
    private final Supplier<ShotParameters> movingShot;

    public ShootCommands(
            final Swerve swerve,
            final Intake intake,
            final Indexer indexer,
            final FuelState fuelState,
            final Superstructure superstructure
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.indexer = indexer;
        this.fuelState = fuelState;
        this.superstructure = superstructure;

        this.targetSupplier = () -> getTarget(superstructure.getTurretTranslation(swerve.getPose()));
        this.targetPoseSupplier = getTargetPoseSupplier();

        this.staticShotProvider = new StaticShot();
        this.staticShot = staticShotProvider.parametersSupplier(
                swerve::getPose,
                superstructure::getRobotToTurret,
                swerve::getRobotRelativeSpeeds,
                targetSupplier,
                targetPoseSupplier
        );

        this.movingShotProvider = new MovingTOFShot();
        this.movingShot = movingShotProvider.parametersSupplier(
                swerve::getPose,
                superstructure::getRobotToTurret,
                swerve::getRobotRelativeSpeeds,
                targetSupplier,
                targetPoseSupplier
        );
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LogKey + "/Target", targetSupplier.get());
        Logger.recordOutput(
                LogKey + "/DistanceToTarget",
                superstructure.getTurretTranslation(swerve.getPose())
                        .getDistance(targetPoseSupplier.get().getTranslation())
        );
    }

    public static double linearSpeed(final ChassisSpeeds speeds) {
        return Math.hypot(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond
        );
    }

    public Command trackTarget() {
        final Supplier<SwerveSpeed.Speeds> swerveSpeedsSupplier =
                () -> switch (targetSupplier.get()) {
                    case HUB -> SwerveSpeed.Speeds.SHOOTING;
                    case FERRY, FERRY_BLOCKED -> SwerveSpeed.Speeds.FERRYING;
                };

        final Supplier<ShotParameters> staticParametersSupplier = staticShotProvider.parametersSupplier(
                swerve::getPose,
                superstructure::getRobotToTurret,
                swerve::getRobotRelativeSpeeds,
                targetSupplier,
                targetPoseSupplier
        );
        final Supplier<ShotParameters> movingTOFParametersSupplier = movingShotProvider.parametersSupplier(
                swerve::getPose,
                superstructure::getRobotToTurret,
                () -> {
                    final SwerveSpeed.Speeds speeds = swerveSpeedsSupplier.get();
                    final ChassisSpeeds robotSpeeds = swerve.getRobotRelativeSpeeds();
                    final double linearSpeed = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
                    return robotSpeeds
                            .div(linearSpeed)
                            .times(Math.min(linearSpeed, speeds.getTranslationSpeed()));
                },
                targetSupplier,
                targetPoseSupplier
        );

        return superstructure.runParametersWithHoodStowed(
                () -> linearSpeed(swerve.getFieldRelativeSpeeds()) <= 1e-3
                        ? staticParametersSupplier.get()
                        : movingTOFParametersSupplier.get()
        ).withName("TrackTarget");
    }

    public Command stopAndShoot() {
        final LoggedTrigger targetValid = group.t(
                "TargetValid",
                () -> switch (targetSupplier.get()) {
                    case HUB, FERRY -> true;
                    case FERRY_BLOCKED -> false;
                });

        return deadline(
                repeatingSequence(
                        waitUntil(targetValid.and(superstructure::atSetpoint)),
                        deadline(
                                indexer.feed()
                                        .onlyWhile(targetValid
                                                .and(superstructure.turretAtSetpoint
                                                        .and(superstructure.hoodAtSetpoint)
                                                        .and(superstructure.shooterAtSetpoint
                                                                .debounce(
                                                                        0.5,
                                                                        Debouncer.DebounceType.kFalling
                                                                )))
                                        ),
                                intake.stowFeed()
                        )
                )
                        .onlyIf(fuelState.hasFuel)
                        .onlyWhile(fuelState.hasFuel
                                .or(intake.isIntaking)),
                superstructure.runParameters(staticShot),
                swerve.runWheelXCommand()
        ).withName("StopAndShoot");
    }

    public Command shoot() {
        final LoggedTrigger targetValid = group.t(
                "TargetValid",
                () -> switch (targetSupplier.get()) {
                    case HUB, FERRY -> true;
                    case FERRY_BLOCKED -> false;
                }
        );

        final Supplier<SwerveSpeed.Speeds> swerveSpeedsSupplier =
                () -> switch (targetSupplier.get()) {
                    case HUB -> SwerveSpeed.Speeds.SHOOTING;
                    case FERRY, FERRY_BLOCKED -> SwerveSpeed.Speeds.FERRYING;
                };
        final LoggedTrigger swerveReady = group.t(
                "SwerveReady",
                () -> linearSpeed(swerve.getFieldRelativeSpeeds())
                        <= swerveSpeedsSupplier.get().getTranslationSpeed() + SwerveSpeedTolerance
        );

        return deadline(
                repeatingSequence(
                        waitUntil(targetValid
                                .and(swerveReady)
                                .and(superstructure::atSetpoint)),
                        deadline(
                                Commands.select(Map.of(
                                        Target.HUB,
                                        indexer.feed()
                                                .onlyWhile(targetValid
                                                        .and(swerveReady)
                                                        .and(superstructure.turretAtSetpoint
                                                                .and(superstructure.hoodAtSetpoint
                                                                        .debounce(
                                                                                0.05,
                                                                                Debouncer.DebounceType.kFalling
                                                                        ))
                                                                .and(superstructure.shooterAtSetpoint
                                                                        .debounce(
                                                                                0.5,
                                                                                Debouncer.DebounceType.kFalling
                                                                        )))),
                                        Target.FERRY,
                                        indexer.feed()
                                                .onlyWhile(targetValid
                                                        .and(swerveReady)
                                                        .and(superstructure.turretAtSetpoint
                                                                .and(superstructure.hoodAtSetpoint
                                                                        .debounce(
                                                                                0.25,
                                                                                Debouncer.DebounceType.kFalling
                                                                        ))
                                                                .and(superstructure.shooterAtSetpoint
                                                                        .debounce(
                                                                                0.5,
                                                                                Debouncer.DebounceType.kFalling
                                                                        )))),
                                        Target.FERRY_BLOCKED,
                                        Commands.none()
                                ), targetSupplier),
                                intake.stowFeed().asProxy()
                                        .unless(intake.isIntaking)
                        )
                )
                        .onlyIf(fuelState.hasFuel)
                        .onlyWhile(fuelState.hasFuel
                                .or(intake.isIntaking)),
                SwerveSpeed.toSwerveSpeed(swerveSpeedsSupplier),
                superstructure.runParameters(movingShot)
        ).withName("Shoot");
    }

    public Command shootNoVision() {
        return deadline(
                repeatingSequence(
                        waitUntil(superstructure::atSetpoint),
                        deadline(
                                indexer.feed()
                                        .onlyWhile(superstructure::atSetpoint),
                                intake.stowFeed().asProxy()
                        )
                ),
                swerve.runWheelXCommand(),
                superstructure.toGoal(Superstructure.Goal.NO_VISION)
        ).withName("ShootNoVision");
    }

    public static Target getTarget(final Translation2d turretTranslation) {
        final double turretX = turretTranslation.getX();
        final double turretY = turretTranslation.getY();

        final double ferryXBoundary = FieldConstants.getFerryXBoundary();
        final boolean isRed = Robot.IsRedAlliance.getAsBoolean();
        final boolean canFerryX = isRed
                ? turretX <= ferryXBoundary
                : turretX >= ferryXBoundary;

        final double ferryLeftBoundary = FieldConstants.getFerryLeftYBoundary();
        final double ferryRightBoundary = FieldConstants.getFerryRightYBoundary();
        final boolean canFerryY = isRed
                ? (turretY >= ferryLeftBoundary || turretY <= ferryRightBoundary)
                : (turretY <= ferryLeftBoundary || turretY >= ferryRightBoundary);

        return canFerryX
                ? (canFerryY ? Target.FERRY : Target.FERRY_BLOCKED)
                : Target.HUB;
    }

    public static Supplier<Pose2d> getTargetPoseSupplier(
            final Supplier<Translation2d> turretTranslationSupplier,
            final Function<Translation2d, Target> targetFunction
    ) {
        return () -> {
            final Translation2d turretTranslation = turretTranslationSupplier.get();
            final Target target = targetFunction.apply(turretTranslation);
            return switch (target) {
                case HUB -> FieldConstants.getHubPose();
                case FERRY, FERRY_BLOCKED -> {
                    final boolean isRed = Robot.IsRedAlliance.getAsBoolean();
                    final Pose2d ferryLeft = FieldConstants.getFerryLeft();
                    final Pose2d ferryRight = FieldConstants.getFerryRight();

                    yield turretTranslation.getY() <= FieldConstants.getFerryLeftYBoundary()
                            ? (isRed ? ferryRight : ferryLeft)
                            : (isRed ? ferryLeft : ferryRight);
                }
            };
        };
    }

    private Supplier<Pose2d> getTargetPoseSupplier() {
        return getTargetPoseSupplier(
                () -> superstructure.getTurretTranslation(swerve.getPose()),
                ShootCommands::getTarget
        );
    }
}