package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.PoseConstants;

import java.util.function.Supplier;

public class ShotCalculator {
    private record ShooterCalculation(
            double shooterVelocityRotsPerSec,
            double hoodPositionRots
    ) implements Interpolatable<ShooterCalculation> {
        @Override
        public ShooterCalculation interpolate(final ShooterCalculation endValue, final double t) {
            return new ShooterCalculation(
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

    public static final InterpolatingTreeMap<Double, ShooterCalculation> ShotMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShooterCalculation::interpolate);
    static {
        ShotMap.put(1.30, new ShooterCalculation(28, 0));
        ShotMap.put(2.07, new ShooterCalculation(32, 0.005));
        ShotMap.put(2.42, new ShooterCalculation(32, 0.008));
        ShotMap.put(2.67, new ShooterCalculation(33, 0.01));
        ShotMap.put(3.07, new ShooterCalculation(35, 0.012));
        ShotMap.put(3.60, new ShooterCalculation(36, 0.014));
        ShotMap.put(4.54, new ShooterCalculation(40, 0.021));
        ShotMap.put(5.18, new ShooterCalculation(43, 0.025));
        ShotMap.put(5.84, new ShooterCalculation(44, 0.027));
        ShotMap.put(6.16, new ShooterCalculation(44, 0.041));
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
            double turretRotationRots,
            double turretSpeedRotsPerSec,
            double hoodRotationRots,
            double shooterVelocityRotsPerSec
    ) {}

    private static final double LookaheadSeconds = 0.025;

    public static Supplier<ShotCalculation> getStaticShotCalculationSupplier(
            final Supplier<Pose2d> robotPoseSupplier,
            final Supplier<ChassisSpeeds> fieldRelativeSwerveSpeedsSupplier,
            final Supplier<Pose2d> targetPoseSupplier
    ) {
        return () -> getStaticShotCalculation(
                robotPoseSupplier.get(),
                fieldRelativeSwerveSpeedsSupplier.get(),
                targetPoseSupplier.get()
        );
    }

    public static ShotCalculation getStaticShotCalculation(
            final Pose2d robotPose,
            final ChassisSpeeds swerveSpeeds,
            final Pose2d targetPose
    ) {
        final Pose2d turretPose = robotPose.transformBy(PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D);

        final Translation2d targetTranslation = targetPose.getTranslation();

        final double turretToTargetDistance = targetTranslation.getDistance(turretPose.getTranslation());

        final Rotation2d desiredTurretAngle = targetTranslation.minus(turretPose.getTranslation())
                .getAngle()
                .minus(robotPose.getRotation());

        final ShooterCalculation shooterCalculation = ShotMap.get(turretToTargetDistance);
        return new ShotCalculation(
                desiredTurretAngle.getRadians(),
                -Units.radiansToRotations(swerveSpeeds.omegaRadiansPerSecond),
                shooterCalculation.hoodPositionRots,
                shooterCalculation.shooterVelocityRotsPerSec
        );
    }

    public static Supplier<ShotCalculation> getMovingShotCalculationSupplier(
            final Supplier<Pose2d> robotPoseSupplier,
            final Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
            final Supplier<Pose2d> targetPoseSupplier
    ) {
        return () -> getMovingShotCalculation(
                robotPoseSupplier.get(),
                robotRelativeSpeedsSupplier.get(),
                targetPoseSupplier.get()
        );
    }

    private static ShotCalculation getMovingShotCalculation(
            final Pose2d swervePose,
            final ChassisSpeeds swerveSpeeds,
            final Pose2d targetPose
    ) {
        final Pose2d turretPose = swervePose.transformBy(PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D);

        final Translation2d targetTranslation = targetPose.getTranslation();

        final double turretToTargetDistance = targetTranslation.getDistance(turretPose.getTranslation());

        final Pose2d lookaheadRobotPose = swervePose.exp(new Twist2d(
                swerveSpeeds.vxMetersPerSecond * LookaheadSeconds,
                swerveSpeeds.vyMetersPerSecond * LookaheadSeconds,
                swerveSpeeds.omegaRadiansPerSecond * LookaheadSeconds
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

        final ShooterCalculation shotCalculation = ShotMap.get(turretToTargetDistance);
        return new ShotCalculation(
                desiredTurretAngle.getRotations(),
                -Units.radiansToRotations(swerveSpeeds.omegaRadiansPerSecond),
                shotCalculation.hoodPositionRots(),
                shotCalculation.shooterVelocityRotsPerSec()
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