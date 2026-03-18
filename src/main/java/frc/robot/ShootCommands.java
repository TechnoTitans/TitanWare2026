package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.PoseConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.ShotCalculator;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.commands.LoggedTrigger;
import frc.robot.utils.subsystems.VirtualSubsystem;
import frc.robot.utils.teleop.SwerveSpeed;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ShootCommands extends VirtualSubsystem {
    protected static final String LogKey = "ShootCommands";
    public static final int BackOutCurrentThreshold = 48;

    public enum Target {
        HUB,
        FERRY,
        FERRY_BLOCKED
    }

    private final Swerve swerve;
    private final Superstructure superstructure;
    private final Intake intake;
    private final Indexer indexer;

    private final LoggedTrigger.Group group;
    private final LoggedTrigger shouldBackOutFeeder;

    public ShootCommands(
            final Swerve swerve,
            final Superstructure superstructure,
            final Intake intake,
            final Indexer indexer
    ) {
        this.swerve = swerve;
        this.superstructure = superstructure;
        this.intake = intake;
        this.indexer = indexer;

        group = LoggedTrigger.Group.from(LogKey);

        this.shouldBackOutFeeder = group.t(
                "ShouldBackOutFeeder",
                () -> indexer.getFeederFilteredCurrent() > BackOutCurrentThreshold
        ).debounce(0.25);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Target",
                getTarget(swerve.getPose().transformBy(PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D)));
    }

    public static double linearSpeed(final ChassisSpeeds speeds) {
        return Math.hypot(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond
        );
    }

    public Command trackTarget() {
        final Supplier<Pose2d> targetPoseSupplier = getTargetPoseSupplier();
        final Supplier<ShotCalculator.ShotCalculation> movingCalculationSupplier =
                ShotCalculator.getMovingShotCalculationSupplier(
                        swerve::getPose,
                        swerve::getRobotRelativeSpeeds,
                        targetPoseSupplier
                );

        return superstructure.runParametersWithHoodStowed(
                movingCalculationSupplier
        ).withName("TrackTarget");
    }

    public Command shoot() {
        final Supplier<Pose2d> targetSupplier = getTargetPoseSupplier();
        final LoggedTrigger targetValid = group.t(
                "TargetValid",
                () -> switch (
                        getTarget(swerve.getPose().transformBy(PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D))
                        ) {
                    case HUB, FERRY -> true;
                    case FERRY_BLOCKED -> false;
                }
        );

        final LoggedTrigger swerveReady = group.t(
                "SwerveReady",
                () -> linearSpeed(swerve.getFieldRelativeSpeeds())
                        <= SwerveSpeed.Speeds.SHOOTING.getTranslationSpeed()
        );

        return Commands.deadline(
                Commands.repeatingSequence(
                        Commands.waitUntil(targetValid
                                .and(swerveReady)
                                .and(superstructure.atSetpoint)),
                        Commands.deadline(
                                indexer.feed()
                                        .onlyWhile(targetValid
                                                .and(swerveReady)
                                                .and(superstructure.atSetpoint)),
                                intake.stowFeed().asProxy()
                        )
//                        indexer.backOut()
                ),
                SwerveSpeed.toSwerveSpeed(SwerveSpeed.Speeds.SHOOTING),
                superstructure.runParameters(
                        ShotCalculator.getMovingShotCalculationSupplier(
                                swerve::getPose,
                                swerve::getRobotRelativeSpeeds,
                                targetSupplier
                        )
                )
        ).withName("Shoot");
    }

    public static Target getTarget(final Pose2d turretPose) {
        final double turretX = turretPose.getX();
        final double turretY = turretPose.getY();

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

    private Supplier<Pose2d> getTargetPoseSupplier() {
        return () -> {
            final Pose2d robotPose = swerve.getPose();
            final Target target = getTarget(robotPose);
            return switch (target) {
                case HUB -> FieldConstants.getHubPose();
                case FERRY, FERRY_BLOCKED -> {
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
}