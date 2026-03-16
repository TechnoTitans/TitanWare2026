package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.RobotCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.PoseConstants;

import java.util.function.Supplier;

public class ShotCalculator {
    public enum Target {
        HUB,
        FERRYING;
    }
    public record ShotCalc(
            double shooterVelocityRotsPerSec,
            double hoodPositionRots
    ) implements Interpolatable<ShotCalc> {
        @Override
        public ShotCalc interpolate(final ShotCalc endValue, final double t) {
            return new ShotCalc(
                    MathUtil.interpolate(
                            shooterVelocityRotsPerSec,
                            endValue.shooterVelocityRotsPerSec,
                            t
                    ),
                    MathUtil.interpolate(
                            hoodPositionRots,
                            endValue.hoodPositionRots,
                            t
                    )
            );
        }
    }

    public static final InterpolatingTreeMap<Double, ShotCalc> ShotMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotCalc::interpolate);
    static {
        ShotMap.put(1.30, new ShotCalc(28, 0));
        ShotMap.put(2.07, new ShotCalc(32, 0.005));
        ShotMap.put(2.42, new ShotCalc(32, 0.008));
        ShotMap.put(2.67, new ShotCalc(33, 0.01));
        ShotMap.put(3.07, new ShotCalc(35, 0.012));
        ShotMap.put(3.60, new ShotCalc(36, 0.014));
        ShotMap.put(4.54, new ShotCalc(40, 0.021));
        ShotMap.put(5.18, new ShotCalc(43, 0.025));
        ShotMap.put(5.84, new ShotCalc(44, 0.027));
        ShotMap.put(6.16, new ShotCalc(44, 0.041));
    }

    private static final InterpolatingDoubleTreeMap TOFMap = new InterpolatingDoubleTreeMap();
    static {
        TOFMap.put(3.213d, 1.01);
        TOFMap.put(3.664d, 1.19);
        TOFMap.put(4.643, 1.21);
        TOFMap.put(3.17, 1.13);
        TOFMap.put(2.209, 1.15);
        TOFMap.put(3.715, 1.08);
        TOFMap.put(1.989, 1.2);
        TOFMap.put(2.823, 1.18);
    }

    public record ShotCalculation(
            Rotation2d desiredTurretRotation,
            double desiredHoodRotationRots,
            double desiredShooterVelocity,
            ShotCalculator.Target target
    ) {}


    private static final double FerryXBoundary = Units.inchesToMeters(200);
    private static final double DelayTimeSec = 0.025;

    public static ShotCalculation getShotCalculation(
            final Supplier<Pose2d> swervePoseSupplier,
            final Supplier<RobotCommands.ScoringMode> scoringModeSupplier,
            final Supplier<ChassisSpeeds> fieldRelativeSwerveSpeedsSupplier
    ) {
        final RobotCommands.ScoringMode scoringMode = scoringModeSupplier.get();

        return switch (scoringMode) {
            case Stationary -> getShotCalculation(swervePoseSupplier.get(), fieldRelativeSwerveSpeedsSupplier.get());
            case Moving -> getMovingShotCalculation(swervePoseSupplier.get(), fieldRelativeSwerveSpeedsSupplier.get());
        };
    }

    public static ShotCalculation getShotCalculationFromPose(final Pose2d robotPose) {
        final Pose2d turretPose = robotPose.transformBy(PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D);

        final Target target = turretPose.getX() > FerryXBoundary
                ? Target.FERRYING
                : Target.HUB;

        final Translation2d targetTranslation = switch (target) {
            case HUB -> FieldConstants.getHubTarget();
            case FERRYING -> FieldConstants.getFerryingTarget(turretPose.getY());
        };

        final double turretToTargetDistance = targetTranslation.getDistance(turretPose.getTranslation());

        final Rotation2d desiredTurretAngle = targetTranslation.minus(turretPose.getTranslation())
                .getAngle()
                .minus(robotPose.getRotation());

        final ShotCalc shotCalc = ShotMap.get(turretToTargetDistance);
        return new ShotCalculation(
                desiredTurretAngle,
                shotCalc.hoodPositionRots(),
                shotCalc.shooterVelocityRotsPerSec(),
                target
        );
    }

    private static ShotCalculation getShotCalculation(
            final Pose2d robotPose,
            final ChassisSpeeds swerveSpeeds
    ) {
        final Pose2d turretPose = robotPose.transformBy(PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D)
                .exp(new Twist2d(
                        swerveSpeeds.vxMetersPerSecond * DelayTimeSec,
                        swerveSpeeds.vyMetersPerSecond * DelayTimeSec,
                        swerveSpeeds.omegaRadiansPerSecond * DelayTimeSec
                ));

        final Pose2d calculationPose = Robot.IsRedAlliance.getAsBoolean()
                ? turretPose.relativeTo(FieldConstants.RED_ORIGIN)
                : turretPose;

        final Target target = calculationPose.getX() > FerryXBoundary
                ? Target.FERRYING
                : Target.HUB;

        final Translation2d targetTranslation = switch (target) {
            case HUB -> FieldConstants.getHubTarget();
            case FERRYING -> FieldConstants.getFerryingTarget(calculationPose.getY());
        };

        final double turretToTargetDistance = targetTranslation.getDistance(turretPose.getTranslation());

        final Rotation2d desiredTurretAngle = targetTranslation.minus(turretPose.getTranslation())
                .getAngle()
                .minus(robotPose.getRotation());

        final ShotCalc shotCalc = ShotMap.get(turretToTargetDistance);
        return new ShotCalculation(
                desiredTurretAngle,
                shotCalc.hoodPositionRots(),
                shotCalc.shooterVelocityRotsPerSec(),
                target
        );
    }

    private static ShotCalculation getMovingShotCalculation(
            final Pose2d swervePose,
            final ChassisSpeeds swerveSpeeds
    ) {
        final Pose2d turretPose = swervePose.transformBy(PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D);

        final Pose2d calculationPose = Robot.IsRedAlliance.getAsBoolean()
                ? turretPose.relativeTo(FieldConstants.RED_ORIGIN)
                : turretPose;

        final Target target = calculationPose.getX() > FerryXBoundary
                ? Target.FERRYING : Target.HUB;

        final Translation2d targetTranslation = switch (target) {
            case HUB -> FieldConstants.getHubTarget();
            case FERRYING -> FieldConstants.getFerryingTarget(calculationPose.getY());
        };

        final double turretToTargetDistance = targetTranslation.getDistance(turretPose.getTranslation());

        final Pose2d lookaheadRobotPose = swervePose.exp(new Twist2d(
                swerveSpeeds.vxMetersPerSecond * DelayTimeSec,
                swerveSpeeds.vyMetersPerSecond * DelayTimeSec,
                swerveSpeeds.omegaRadiansPerSecond * DelayTimeSec
        ));

        final Rotation2d robotAngle = lookaheadRobotPose.getRotation();
        final ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                swerveSpeeds,
                robotAngle
        );

        final Transform2d turretOffset = turretPose.minus(lookaheadRobotPose);
        final ChassisSpeeds turretFieldVelocity = getTurretFieldSpeeds(lookaheadRobotPose, turretOffset, fieldSpeeds);

        double timeOfFlight;
        Pose2d futureTurretPose = turretPose;
        double futureDistance = turretToTargetDistance;
        for (int i = 0; i < 20; i++) {
            timeOfFlight = TOFMap.get(futureDistance);

            final Translation2d delta = new Translation2d(
                    turretFieldVelocity.vxMetersPerSecond * timeOfFlight,
                    turretFieldVelocity.vyMetersPerSecond * timeOfFlight
            );
            futureTurretPose = new Pose2d(
                    turretPose.getTranslation().plus(delta),
                    turretPose.getRotation()
            );
            futureDistance = futureTurretPose
                    .getTranslation()
                    .getDistance(targetTranslation);
        }

        final Pose2d futureRobotPose = futureTurretPose.transformBy(turretOffset.inverse());
        final Rotation2d desiredTurretAngle = targetTranslation.minus(futureTurretPose.getTranslation())
                .getAngle()
                .minus(futureRobotPose.getRotation());

        final ShotCalc shotCalc = ShotMap.get(turretToTargetDistance);
        return new ShotCalculation(
                desiredTurretAngle,
                shotCalc.hoodPositionRots(),
                shotCalc.shooterVelocityRotsPerSec(),
                target
        );
    }

    public static ChassisSpeeds getTurretFieldSpeeds(
            final Pose2d robotPose,
            final Translation2d turretTranslation,
            final ChassisSpeeds fieldRelativeSpeeds
    ) {
        return getTurretFieldSpeeds(
                robotPose,
                new Transform2d(robotPose, new Pose2d(turretTranslation, Rotation2d.kZero)),
                fieldRelativeSpeeds
        );
    }

    private static ChassisSpeeds getTurretFieldSpeeds(
            final Pose2d robotPose,
            final Transform2d turretOffset,
            final ChassisSpeeds fieldRelativeSpeeds
    ) {
        final double omega = fieldRelativeSpeeds.omegaRadiansPerSecond;
        final Translation2d rField = turretOffset
                .getTranslation()
                .rotateBy(robotPose.getRotation());

        final double tangentVx = -omega * rField.getY();
        final double tangentVy = omega * rField.getX();

        return new ChassisSpeeds(
                fieldRelativeSpeeds.vxMetersPerSecond + tangentVx,
                fieldRelativeSpeeds.vyMetersPerSecond + tangentVy,
                omega
        );
    }
}