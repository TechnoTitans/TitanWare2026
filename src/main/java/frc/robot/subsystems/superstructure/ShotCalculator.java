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
import frc.robot.constants.HardwareConstants;

import java.util.function.Supplier;

public class ShotCalculator {
    public enum Target {
        HUB,
        FERRYING
    }

    public record ShotParameters(
            Shooter shooter,
            Rotation2d turretAngle,
            double turretVelocityRotsPerSec
    ) {
        public record Shooter(
                double shooterVelocityRotsPerSec,
                double hoodPositionRots
        ) implements Interpolatable<Shooter> {
            @Override
            public Shooter interpolate(final Shooter endValue, final double t) {
                return new Shooter(
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
    }

    private static final InterpolatingTreeMap<Double, ShotParameters.Shooter> shotMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotParameters.Shooter::interpolate);
    static {
        shotMap.put(1.7696d, new ShotParameters.Shooter(
                30,
                0
        ));

        shotMap.put(2.4459, new ShotParameters.Shooter(
                30.55,
        0.0056
        ));

        shotMap.put(2.9222d, new ShotParameters.Shooter(
                35,
                0.0056
        ));

        shotMap.put(3.4454d, new ShotParameters.Shooter(
                35,
                0.0264
        ));

        shotMap.put(3.664d, new ShotParameters.Shooter(
                35,
            0.0444
        ));

        shotMap.put(3.7005d, new ShotParameters.Shooter(
                35,
                0.0537
        ));

        shotMap.put(3.973d, new ShotParameters.Shooter(
                35,
                0.05
        ));

        shotMap.put(4.9717d, new ShotParameters.Shooter(
                40,
                0.0583
        ));

        shotMap.put(5.43d, new ShotParameters.Shooter(
                40,
                0.0583
        ));
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
            double desiredHoodRotation,
            double desiredShooterVelocityRotsPerSec,
            ShotCalculator.Target target
    ) {}

    private static final double FerryXBoundary = Units.inchesToMeters(200);
    private static final double DelayTimeSec = 0.01;
    private static final Rotation2d wrapOffset = Rotation2d.fromRotations(0.75);

    public static ShotCalculation getShotCalculation(
            final Supplier<Pose2d> swervePoseSupplier,
            final Supplier<RobotCommands.ScoringMode> scoringModeSupplier,
            final Supplier<ChassisSpeeds> fieldRelativeSwerveSpeedsSupplier
    ) {
        final RobotCommands.ScoringMode scoringMode = scoringModeSupplier.get();

        final Pose2d robotPose = swervePoseSupplier.get();
        return switch (scoringMode) {
            case Stationary -> getShotCalculation(swervePoseSupplier.get());
            case Moving -> getMovingShotCalculation(
                    robotPose,
                    robotPose.plus(HardwareConstants.TURRET.offsetFromCenter()).getTranslation(),
                    fieldRelativeSwerveSpeedsSupplier.get()
            );
        };
    }

    private static ShotCalculation getShotCalculation(final Pose2d robotPose) {
        final Pose2d turretPose = robotPose.plus(HardwareConstants.TURRET.offsetFromCenter());

        final Pose2d alliancedTurretPose = Robot.IsRedAlliance.getAsBoolean()
                ? turretPose.relativeTo(FieldConstants.RED_ORIGIN)
                : turretPose;

        final Target target = alliancedTurretPose.getX() > FerryXBoundary ? Target.FERRYING : Target.HUB;

        final Translation2d targetTranslation =
                switch (target) {
                    case HUB -> FieldConstants.getHubTarget();
                    case FERRYING -> FieldConstants.getFerryingTarget(turretPose.getY());
                };

        final double turretToTargetDistance = targetTranslation.getDistance(turretPose.getTranslation());
        final ShotParameters.Shooter hoodShooterCalculation = shotMap.get(turretToTargetDistance);

        final Rotation2d desiredTurretAngle = targetTranslation.minus(turretPose.getTranslation()).getAngle()
                .minus(robotPose.getRotation());

        return new ShotCalculation(
                desiredTurretAngle,
                hoodShooterCalculation.hoodPositionRots(),
                hoodShooterCalculation.shooterVelocityRotsPerSec(),
                target
        );
    }

    private static ShotCalculation getMovingShotCalculation(
            final Pose2d swervePose,
            final Translation2d turretTranslation,
            final ChassisSpeeds swerveSpeeds
    ) {
        final Pose2d turretPose = new Pose2d(turretTranslation, Rotation2d.kZero);

        final Pose2d alliancedTurretPose = Robot.IsRedAlliance.getAsBoolean()
                ? turretPose.relativeTo(FieldConstants.RED_ORIGIN)
                : turretPose;

        final Target target = alliancedTurretPose.getX() > FerryXBoundary
                ? Target.FERRYING : Target.HUB;

        final Translation2d targetTranslation =
                switch (target) {
                    case HUB -> FieldConstants.getHubTarget();
                    case FERRYING -> FieldConstants.getFerryingTarget(turretPose.getY());
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

        final Pose2d futureRobotPose = futureTurretPose.transformBy(turretOffset.inverse());
        final Rotation2d desiredTurretAngle = targetTranslation.minus(turretPose.getTranslation()).getAngle().minus(
                turretPose.getRotation()
        );

        return new ShotCalculation(
                desiredTurretAngle,
                shotMap.get(futureDistance).hoodPositionRots(),
                shotMap.get(futureDistance).shooterVelocityRotsPerSec(),
                target
        );
    }

    private static ChassisSpeeds getTurretFieldSpeeds(
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
