package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.FuelState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.MovingTOFShot;
import frc.robot.subsystems.superstructure.ShotParameters;
import frc.robot.subsystems.superstructure.StaticShot;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.Container;
import frc.robot.utils.commands.LoggedTrigger;
import frc.robot.utils.teleop.SwerveSpeed;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class ShootCommands {
    protected static final String LogKey = "ShootCommands";
    private static final SwerveSpeed.Speeds ShootAndScootSpeeds = SwerveSpeed.Speeds.SHOOT_AND_SCOOT;
    private static final double ShootAndScootTolerance = 0.25;
    public enum Target {
        HUB,
        FERRY,
        NONE_FERRY_BLOCKED
    }

    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final Swerve swerve;
    private final Intake intake;
    private final Indexer indexer;
    private final FuelState fuelState;
    private final Superstructure superstructure;

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
    }

    public static double linearSpeed(final ChassisSpeeds speeds) {
        return Math.hypot(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond
        );
    }

    public static Target getTarget(final Pose2d robotPose) {
        final double robotX = robotPose.getX();
        final double robotY = robotPose.getY();

        final double ferryXBoundary = FieldConstants.getFerryXBoundary();
        final boolean isRed = Robot.IsRedAlliance.getAsBoolean();
        final boolean canFerryX = isRed
                ? robotX <= ferryXBoundary
                : robotX >= ferryXBoundary;

        final double ferryLeftBoundary = FieldConstants.getFerryLeftYBoundary();
        final double ferryRightBoundary = FieldConstants.getFerryRightYBoundary();
        final boolean canFerryY = isRed
                ? (robotY >= ferryLeftBoundary || robotY <= ferryRightBoundary)
                : (robotY <= ferryLeftBoundary || robotY >= ferryRightBoundary);

        return canFerryX
                ? (canFerryY ? Target.FERRY : Target.NONE_FERRY_BLOCKED)
                : Target.HUB;
    }

    private Supplier<Pose2d> getTargetPoseSupplier() {
        return () -> {
            final Pose2d robotPose = swerve.getPose();
            final Target target = getTarget(robotPose);
            return switch (target) {
                case HUB -> FieldConstants.getHubPose();
                case FERRY, NONE_FERRY_BLOCKED -> {
                    final boolean isRed = Robot.IsRedAlliance.getAsBoolean();
                    final Pose2d ferryLeft = FieldConstants.getFerryLeft();
                    final Pose2d ferryRight = FieldConstants.getFerryRight();

                    yield robotPose.getY() <= FieldConstants.getFerryLeftYBoundary()
                            ? (isRed ? ferryRight : ferryLeft)
                            : (isRed ? ferryLeft : ferryRight);
                }
            };
        };
    }

    public Command trackTarget() {
        final Supplier<Pose2d> targetSupplier = getTargetPoseSupplier();
        final Supplier<ShotParameters> staticParametersSupplier = StaticShot.parametersSupplier(
                swerve::getPose,
                superstructure::getTurretTranslation,
                swerve::getRobotRelativeSpeeds,
                targetSupplier
        );
        final Supplier<ShotParameters> movingTOFParametersSupplier = MovingTOFShot.parametersSupplier(
                swerve::getPose,
                superstructure::getTurretTranslation,
                () -> {
                    final ChassisSpeeds robotSpeeds = swerve.getRobotRelativeSpeeds();
                    final double linearSpeed = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
                    return robotSpeeds
                            .div(linearSpeed)
                            .times(Math.min(linearSpeed, ShootAndScootSpeeds.getTranslationSpeed()));
                },
                targetSupplier
        );

        return superstructure.runParametersHoodStowed(
                () -> linearSpeed(swerve.getFieldRelativeSpeeds()) <= 1e-3
                        ? staticParametersSupplier.get()
                        : movingTOFParametersSupplier.get()
        ).withName("TrackTarget");
    }

    public Command stopAndShoot() {
        final Supplier<Pose2d> targetPoseSupplier = getTargetPoseSupplier();
        final LoggedTrigger targetValid = group.t(
                "TargetValid",
                () -> switch (getTarget(swerve.getPose())) {
                    case HUB, FERRY -> true;
                    case NONE_FERRY_BLOCKED -> false;
                });

        return deadline(
                repeatingSequence(
                        waitUntil(targetValid.and(superstructure::atSetpoint)),
                        Commands.deadline(
                                indexer.toFeed()
                                        .onlyWhile(targetValid.and(superstructure::atSetpoint)),
                                intake.stowFeed()
                        )
                )
                        .onlyIf(fuelState.hasFuel)
                        .onlyWhile(fuelState.hasFuel
                                .or(intake.isIntaking)),
                superstructure.runParameters(
                        StaticShot.parametersSupplier(
                                swerve::getPose,
                                superstructure::getTurretTranslation,
                                swerve::getRobotRelativeSpeeds,
                                targetPoseSupplier
                        )
                ),
                swerve.runWheelXCommand()
        ).withName("StopAndShoot");
    }

    public Command shoot() {
        final Supplier<Pose2d> targetSupplier = getTargetPoseSupplier();
        final LoggedTrigger targetValid = group.t(
                "TargetValid",
                () -> switch (getTarget(swerve.getPose())) {
                    case HUB, FERRY -> true;
                    case NONE_FERRY_BLOCKED -> false;
                });
        final Container<SwerveSpeed.Speeds> originalSpeeds = Container.empty();
        final LoggedTrigger swerveReady = group.t(
                "SwerveReady",
                () -> linearSpeed(swerve.getFieldRelativeSpeeds())
                        <= ShootAndScootSpeeds.getTranslationSpeed() + ShootAndScootTolerance
        );

        return deadline(
                repeatingSequence(
                        waitUntil(targetValid
                                .and(swerveReady)
                                .and(superstructure::atSetpoint)),
                        Commands.deadline(
                                indexer.toFeed()
                                        .onlyWhile(targetValid
                                                .and(swerveReady)
                                                .and(superstructure::atSetpoint)),
                                intake.stowFeed().asProxy()
                                        .unless(intake.isIntaking)
                        )
                )
                        .onlyIf(fuelState.hasFuel)
                        .onlyWhile(fuelState.hasFuel
                                .or(intake.isIntaking)),
                Commands.sequence(
                        originalSpeeds.setCommand(SwerveSpeed.getSwerveSpeed()),
                        Commands.startEnd(
                                () -> SwerveSpeed.setSwerveSpeed(ShootAndScootSpeeds),
                                () -> SwerveSpeed.setSwerveSpeed(originalSpeeds.get())
                        )
                ),
                superstructure.runParameters(
                        MovingTOFShot.parametersSupplier(
                                swerve::getPose,
                                superstructure::getTurretTranslation,
                                swerve::getRobotRelativeSpeeds,
                                targetSupplier
                        )
                )
        ).withName("Shoot");
    }
}
