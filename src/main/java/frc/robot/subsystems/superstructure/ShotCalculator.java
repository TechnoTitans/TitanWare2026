package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.PoseConstants;

import java.util.function.Supplier;

//TODO: If ball hit hub, no shoot
public class ShotCalculator {
    public enum Target {
        HUB,
        FERRYING;
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

    private static final InterpolatingDoubleTreeMap TOFMap = new InterpolatingDoubleTreeMap();
    static {
//        TOFMap.put(2.0952d, 1.0);
//        TOFMap.put(2.841d, 1.1);
//        TOFMap.put(3.875d, 1.3);
//        TOFMap.put(5.0058d, 1.5);
//        TOFMap.put(4.4723d, 1.5);
//        TOFMap.put(5.19d, 1.6);
//        TOFMap.put(4.36d, 1.0);


        TOFMap.put(3.213d, 1.01);
        TOFMap.put(3.664d, 1.19);
        TOFMap.put(4.643, 1.21);
        TOFMap.put(3.17, 1.13);
        TOFMap.put(2.209, 1.15);
        TOFMap.put(3.715, 1.08);
        TOFMap.put(1.989, 1.2);
        TOFMap.put(2.823, 1.18);

        // Might not need values
//        shotDataMap.put(2.0952d, new HoodShooterCalculation(
//                Rotation2d.fromDegrees(0.79),
//                30,
//                1
//
//        ));
//
//        shotDataMap.put(2.841d, new HoodShooterCalculation(
//                Rotation2d.fromDegrees(1.6),
//                30,
//                1.1
//
//        ));
//
//        shotDataMap.put(3.875, new HoodShooterCalculation(
//                Rotation2d.fromDegrees(19.4238),
//                36,
//                1.3
//
//        ));
//
//        shotDataMap.put(5.0058, new HoodShooterCalculation(
//                Rotation2d.fromDegrees(20.65428),
//                40,
//                1.5
//
//        ));
//
//        shotDataMap.put(4.4723, new HoodShooterCalculation(
//                Rotation2d.fromDegrees(19.8512),
//                37,
//                1.5
//
//        ));
//
//        shotDataMap.put(5.19, new HoodShooterCalculation(
//                Rotation2d.fromDegrees(20.628),
//                42,
//                1.6
//
//        ));
//
//        shotDataMap.put(4.36, new HoodShooterCalculation(
//                Rotation2d.fromDegrees(20.0736),
//                37,
//                1
//
//        ));



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

        shotDataMap.put(3.664d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(16),
                35,
                1
        ));

        shotDataMap.put(3.973d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(18),
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


    private static final double FerryXBoundary = Units.inchesToMeters(200);
    private static final double DelayTimeSec = 0.025;

    private static final double TurretZeroOffset = 0.25;
    private static final Rotation2d WrapOffset = Rotation2d.fromRotations(0.75);

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

    public static ShotCalculation getShotCalculationFromPose(
            final Pose2d pose
    ) {
        final Pose2d turretPose = pose.transformBy(PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D);

        final Target target =
                turretPose.getX()
                        > FerryXBoundary ? Target.FERRYING : Target.HUB;

        final Translation2d targetTranslation =
                switch (target) {
                    case HUB -> FieldConstants.getHubTarget();
                    case FERRYING -> FieldConstants.getFerryingTarget(turretPose.getY());
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

    private static ShotCalculation getMovingShotCalculation(
            final Pose2d swervePose,
            final ChassisSpeeds swerveSpeeds
    ) {
        final Pose2d turretPose = swervePose.transformBy(PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D);

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
            futureDistance = futureTurretPose.getTranslation()
                    .getDistance(targetTranslation);
        }

        final Rotation2d desiredTurretAngle = targetTranslation.minus(turretPose.getTranslation()).getAngle().minus(
                turretPose.getRotation()
        );


        return new ShotCalculation(
                wrapTurret(desiredTurretAngle),
                shotDataMap.get(futureDistance).hoodRotation,
                shotDataMap.get(futureDistance).flywheelVelocity,
                target
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

    private static Rotation2d wrapTurret(final Rotation2d desiredTurretRotation) {
        if (desiredTurretRotation.getRotations() > TurretZeroOffset) {
            return desiredTurretRotation.minus(WrapOffset);
        }

        return desiredTurretRotation.rotateBy(Rotation2d.kCCW_90deg);
    }
}