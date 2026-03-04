package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.PoseConstants;

import java.util.function.Supplier;

public class ShotCalculator {
    public enum Target {
        HUB,
        FERRYING
    }

    public record HoodShooterCalculation(
            Rotation2d hoodRotation,
            double flywheelVelocity,
            double shotTime
    ) implements Interpolatable<HoodShooterCalculation> {

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

    private static final InterpolatingTreeMap<Double, HoodShooterCalculation> shotDataMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            HoodShooterCalculation.interpolator
    );

    static {
        shotDataMap.put(1.7696d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(0),
                30,
                1
        ));

        shotDataMap.put(2.4459, new HoodShooterCalculation(
                Rotation2d.fromDegrees(2.016),
                30.55,
                1
        ));

        shotDataMap.put(2.9222d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(2.016),
                35,
                1
        ));

        shotDataMap.put(3.7005d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(19.332),
                35,
                1
        ));

        shotDataMap.put(4.9717d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(21),
                40,
                1
        ));
    }

    public record ShotCalculation(
            Rotation2d desiredTurretRotation,
            Rotation2d desiredHoodRotation,
            double desiredShooterVelocity,
            ShotCalculator.Target target
    ) {}

    private static final double hoodDownLowerXBoundary = Units.inchesToMeters(154);
    private static final double FerryXBoundary = Units.inchesToMeters(200);
    private static final double DelayTimeSec = 0.005;
    private static final Rotation2d wrapOffset = Rotation2d.fromRotations(0.75);
    //TODO: Try to see if this is worth it
//    private static final LinearFilter turretFilter = LinearFilter.movingAverage(5);

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

    //TODO: Add so that hood can't go up if we are going under trench
    private static ShotCalculation getShotCalculation(
            final Pose2d swervePose,
            final ChassisSpeeds swerveSpeeds
    ) {
        final Pose2d turretPose = swervePose.transformBy(PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D)
                .exp(new Twist2d(
                        swerveSpeeds.vxMetersPerSecond * DelayTimeSec,
                        swerveSpeeds.vyMetersPerSecond * DelayTimeSec,
                        swerveSpeeds.omegaRadiansPerSecond * DelayTimeSec
                ));

        final Pose2d calculationPose = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? turretPose : turretPose.relativeTo(FieldConstants.RED_ORIGIN);

        final Target target =
                calculationPose.getX()
                        > FerryXBoundary ? Target.FERRYING : Target.HUB;

        final Translation2d targetTranslation =
                switch (target) {
                    case HUB -> FieldConstants.getHubTarget();
                    case FERRYING -> FieldConstants.getFerryingTarget(calculationPose.getY());
                };

        final double turretToTargetDistance = targetTranslation.getDistance(turretPose.getTranslation());

        final Rotation2d desiredTurretAngle = targetTranslation.minus(turretPose.getTranslation()).getAngle().minus(
                turretPose.getRotation()
        );

        final HoodShooterCalculation hoodShooterCalculation = shotDataMap.get(turretToTargetDistance);

        final Rotation2d hoodRotation = calculationPose.getX() > hoodDownLowerXBoundary && calculationPose.getX() < FerryXBoundary
                ? Rotation2d.kZero : hoodShooterCalculation.hoodRotation;

        return new ShotCalculation(
                wrapTurret(desiredTurretAngle),
                hoodRotation,
                hoodShooterCalculation.flywheelVelocity(),
                target
        );
    }

    //TODO: Needs to be implemented
    private static ShotCalculation getMovingShotCalculation(
            final Pose2d swervePose,
            final ChassisSpeeds swerveSpeeds
    ) {
        final Pose2d turretPose = swervePose.transformBy(PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D)
                .exp(new Twist2d(
                        swerveSpeeds.vxMetersPerSecond * DelayTimeSec,
                        swerveSpeeds.vyMetersPerSecond * DelayTimeSec,
                        swerveSpeeds.omegaRadiansPerSecond * DelayTimeSec
                ));

        final Pose2d calculationPose = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue
                ? turretPose : turretPose.relativeTo(FieldConstants.RED_ORIGIN);

        final Target target =
                calculationPose.getX()
                        > FerryXBoundary ? Target.FERRYING : Target.HUB;

        final Translation2d targetTranslation =
                switch (target) {
                    case HUB -> FieldConstants.getHubTarget();
                    case FERRYING -> FieldConstants.getFerryingTarget(calculationPose.getY());
                };

        final double turretToTargetDistance = targetTranslation.getDistance(turretPose.getTranslation());

        final Rotation2d desiredTurretAngle = targetTranslation.minus(turretPose.getTranslation()).getAngle().minus(
                turretPose.getRotation()
        );

        final HoodShooterCalculation hoodShooterCalculation = shotDataMap.get(turretToTargetDistance);

        final Rotation2d hoodRotation = calculationPose.getX() > hoodDownLowerXBoundary && calculationPose.getX() < FerryXBoundary
                ? Rotation2d.kZero : hoodShooterCalculation.hoodRotation;

        return new ShotCalculation(
                wrapTurret(desiredTurretAngle),
                hoodRotation,
                hoodShooterCalculation.flywheelVelocity(),
                target
        );
    }

    private static Rotation2d wrapTurret(final Rotation2d desiredTurretRotation) {
        if (desiredTurretRotation.getRotations() > 0.25) {
            return desiredTurretRotation.minus(wrapOffset);
        }

        return desiredTurretRotation.rotateBy(Rotation2d.kCCW_90deg);
    }
}
