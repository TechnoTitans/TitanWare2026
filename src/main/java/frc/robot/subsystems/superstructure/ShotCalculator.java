package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.geometry.AllianceFlipUtil;

import java.util.function.Supplier;

public class ShotCalculator {
    public enum Target {
        HUB(FieldConstants.Hub.hubCenterPoint),
        FERRYING(FieldConstants.ferryTarget);

        private final Translation2d targetTranslation;

        Target(final Translation2d targetTranslation) {
            this.targetTranslation = targetTranslation;
        }

        public Translation2d getTargetTranslation() {
            return targetTranslation;
        }
    }

    public enum ScoringType {
        Stationary,
        Moving
    }

    public record HoodShooterCalculation(
            Rotation2d hoodRotation,
            double flywheelVelocity,
            double shotTime
    )  implements Interpolatable<HoodShooterCalculation> {

        public static final Interpolator<HoodShooterCalculation> interpolator = HoodShooterCalculation::interpolate;

        @Override
        public HoodShooterCalculation interpolate(final HoodShooterCalculation endShotCalculation, final double t) {
            return new HoodShooterCalculation(
                    this.hoodRotation.interpolate(endShotCalculation.hoodRotation, t),
                    MathUtil.interpolate(this.flywheelVelocity, endShotCalculation.flywheelVelocity, t),
                    MathUtil.interpolate(this.shotTime, endShotCalculation.shotTime, t)
            );
        }
    }

    protected static final String LogKey = "Superstructure/ShotCalculator";
    private static final double FerryXBoundary = Units.inchesToMeters(205);

    private static final InterpolatingTreeMap<Double, HoodShooterCalculation> shotDataMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            HoodShooterCalculation.interpolator
    );

    private static final LinearFilter turretAngleFilter = LinearFilter.movingAverage(
            5
    );

    //TODO: Temp values

    static {
        shotDataMap.put(1d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(5),
                10,
                1
        ));

        shotDataMap.put(1.5d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(10),
                10,
                1.05
        ));

        shotDataMap.put(2d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(15),
                10,
                1.1
        ));

        shotDataMap.put(2.5d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(18),
                10,
                1.15
        ));

        shotDataMap.put(3d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(19),
                10,
                1.2
        ));

        shotDataMap.put(3.5d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(20),
                10,
                1.25
        ));

        shotDataMap.put(4d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(22),
                10,
                1.3
        ));

        shotDataMap.put(4.5d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(25),
                10,
                1.35
        ));

        shotDataMap.put(5d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(26),
                10,
                1.4
        ));
    }

    public record ShotCalculation(
            Rotation2d desiredTurretRotation,
            HoodShooterCalculation hoodShooterCalculation,
            Target target
    ) {}

    public static ShotCalculation getShotCalculation(
            final Supplier<Pose2d> swervePoseSupplier,
            final Supplier<ChassisSpeeds> robotRelativeChassisSpeedsSupplier,
            final Supplier<ChassisSpeeds> fieldRelativeChassisSpeedsSupplier
    ) {
        return getShotCalculation(
                swervePoseSupplier.get(),
                robotRelativeChassisSpeedsSupplier.get(),
                fieldRelativeChassisSpeedsSupplier.get()
        );
    }

    private static ShotCalculation getShotCalculation(
            final Pose2d swervePose,
            final ChassisSpeeds robotRelativeChassisSpeeds,
            final ChassisSpeeds fieldRelativeChassisSpeeds
    ) {
        final Pose2d turretPose = swervePose.transformBy(SimConstants.Turret.TURRET_TO_ROBOT_TRANSFORM);

        Target target = turretPose.getX() > FerryXBoundary ? Target.FERRYING : Target.HUB;
        final Translation2d targetTranslation = AllianceFlipUtil.apply(target.getTargetTranslation());

        final double turretToTargetDistance = targetTranslation.getDistance(turretPose.getTranslation());

        final Rotation2d desiredTurretAngle = targetTranslation.minus(turretPose.getTranslation()).getAngle().minus(
                turretPose.getRotation().plus(Rotation2d.fromRadians(fieldRelativeChassisSpeeds.omegaRadiansPerSecond * 0.02))
        );

        return new ShotCalculation(
                desiredTurretAngle,
                shotDataMap.get(
                        turretToTargetDistance
                ),
                target
        );
    }

    public static ShotCalculation getMovingShotCalculation(
            final Supplier<Pose2d> swervePoseSupplier,
            final Supplier<ChassisSpeeds> robotRelativeChassisSpeedsSupplier,
            final Supplier<ChassisSpeeds> swerveChassisSpeedsSupplier
    ) {
        return getMovingShotCalculation(
                swervePoseSupplier.get(),
                robotRelativeChassisSpeedsSupplier.get(),
                swerveChassisSpeedsSupplier.get()
        );
    }

    //TODO: Needs to be implemented
    private static ShotCalculation getMovingShotCalculation(
            final Pose2d swervePose,
            final ChassisSpeeds robotRelativeChassisSpeeds,
            final ChassisSpeeds fieldRelativeChassisSpeeds
    ) {
        final Pose2d turretPose = swervePose.transformBy(SimConstants.Turret.TURRET_TO_ROBOT_TRANSFORM);

        Target target = turretPose.getX() > FerryXBoundary ? Target.FERRYING : Target.HUB;
        final Translation2d targetTranslation = AllianceFlipUtil.apply(target.getTargetTranslation());

        final double turretToTargetDistance = targetTranslation.getDistance(turretPose.getTranslation());

        final Rotation2d desiredTurretAngle = targetTranslation.minus(turretPose.getTranslation()).getAngle().minus(
                turretPose.getRotation()
        );

        return new ShotCalculation(
                desiredTurretAngle,
                shotDataMap.get(
                        turretToTargetDistance
                ),
                target
        );
    }
}
