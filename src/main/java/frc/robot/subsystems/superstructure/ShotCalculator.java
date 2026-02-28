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
import org.littletonrobotics.junction.Logger;

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

    public static final double FerryXBoundary = Units.inchesToMeters(305);

    public static final InterpolatingTreeMap<Double, HoodShooterCalculation> shotDataMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            HoodShooterCalculation.interpolator
    );

    static {
        shotDataMap.put(1d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(5),
                30,
                1
        ));

        shotDataMap.put(1.5d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(30),
                30,
                1.05
        ));

        shotDataMap.put(2d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(15),
                30,
                1.1
        ));

        shotDataMap.put(2.5d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(18),
                30,
                1.15
        ));

        shotDataMap.put(3d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(19),
                30,
                1.2
        ));

        shotDataMap.put(3.5d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(30),
                30,
                1.25
        ));

        shotDataMap.put(4d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(22),
                30,
                1.3
        ));

        shotDataMap.put(4.5d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(25),
                30,
                1.30
        ));

        shotDataMap.put(5d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(26),
                30,
                1.4
        ));
    }

    public record ShotCalculation(
            Rotation2d desiredTurretRotation,
            HoodShooterCalculation hoodShooterCalculation,
            ShotCalculator.Target target
    ) {}

    private static final double DelayTimeSec = 0.005;
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

        return new ShotCalculation(
                wrapTurret(desiredTurretAngle),
                shotDataMap.get(
                        turretToTargetDistance
                ),
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

        return new ShotCalculation(
                wrapTurret(desiredTurretAngle),
                shotDataMap.get(
                        turretToTargetDistance
                ),
                target
        );
    }

    private static Rotation2d wrapTurret(final Rotation2d desiredTurretRotation) {
        if (desiredTurretRotation.getRotations() > 0.25) {
            return desiredTurretRotation.minus(Rotation2d.fromRotations(0.75));
        }

        return desiredTurretRotation.rotateBy(Rotation2d.kCCW_90deg);
    }
}
