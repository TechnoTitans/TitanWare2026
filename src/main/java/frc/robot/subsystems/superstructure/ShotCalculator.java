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

    public static final double TURRET_ZERO_OFFSET = 0.25;

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

        //vids
        shotDataMap.put(2.0952d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(0.79),
                30,
                1

        ));

        shotDataMap.put(2.841d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(1.6),
                30,
                1.1

        ));

        shotDataMap.put(3.875, new HoodShooterCalculation(
                Rotation2d.fromDegrees(19.4238),
                36,
                1.3

        ));

        shotDataMap.put(5.0058, new HoodShooterCalculation(
                Rotation2d.fromDegrees(20.65428),
                40,
                1.5

        ));

        shotDataMap.put(4.4723, new HoodShooterCalculation(
                Rotation2d.fromDegrees(19.8512),
                37,
                1.5

        ));

        shotDataMap.put(5.19, new HoodShooterCalculation(
                Rotation2d.fromDegrees(20.628),
                42,
                1.6

        ));

        shotDataMap.put(4.36, new HoodShooterCalculation(
                Rotation2d.fromDegrees(20.0736),
                37,
                1

        ));



        //

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

        shotDataMap.put(3.4454d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(9.5),
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

        shotDataMap.put(5.43d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(21),
                45,
                1
        ));
    }

    public record ShotCalculation(
            Rotation2d desiredTurretRotation,
            Rotation2d desiredHoodRotation,
            double desiredShooterVelocity,
            ShotCalculator.Target target
    ) {}

    private static final double FerryXBoundary = Units.inchesToMeters(200);
    private static final double DelayTimeSec = 0.01;
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

        return new ShotCalculation(
                wrapTurret(desiredTurretAngle),
                hoodShooterCalculation.hoodRotation,
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

        final double robotAngle = swervePose.getRotation().getRadians();
        final double turretVelocityX =
                swerveSpeeds.vxMetersPerSecond
                + swerveSpeeds.omegaRadiansPerSecond *
                        (PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D.getY() * Math.cos(robotAngle))
                            - PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D.getX() * Math.sin(robotAngle);

        final double turretVelocityY =
                swerveSpeeds.vyMetersPerSecond
                    + swerveSpeeds.omegaRadiansPerSecond
                        * (PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D.getX() * Math.cos(robotAngle))
                            - PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D.getY() * Math.sin(robotAngle);

        double tOF;
        Pose2d lookaheadPose;
        double lookAheadTurretToTargetDistance = turretToTargetDistance;

        for (int i = 0; i < 20; i++) {
            tOF = shotDataMap.get(lookAheadTurretToTargetDistance).shotTime;
            double offsetX = turretVelocityX * tOF;
            double offsetY = turretVelocityY * tOF;
            lookaheadPose =
                    new Pose2d(
                            turretPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                            turretPose.getRotation()
                    );

            lookAheadTurretToTargetDistance = targetTranslation.getDistance(lookaheadPose.getTranslation());
        }


        final Rotation2d desiredTurretAngle = targetTranslation.minus(turretPose.getTranslation()).getAngle().minus(
                turretPose.getRotation()
        );

        final HoodShooterCalculation hoodShooterCalculation = shotDataMap.get(turretToTargetDistance);
        return new ShotCalculation(
                wrapTurret(desiredTurretAngle),
                hoodShooterCalculation.hoodRotation,
                hoodShooterCalculation.flywheelVelocity(),
                target
        );
    }

    private static Rotation2d wrapTurret(final Rotation2d desiredTurretRotation) {
        if (desiredTurretRotation.getRotations() > TURRET_ZERO_OFFSET) {
            return desiredTurretRotation.minus(wrapOffset);
        }

        return desiredTurretRotation.rotateBy(Rotation2d.kCCW_90deg);
    }
}
