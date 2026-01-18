package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SimConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ShotCalculator {
    public enum Target {
        HUB(FieldConstants.hubCenter),
        FERRYING(FieldConstants.ferryTarget);

        private final Translation2d targetTranslation;

        Target(final Translation2d targetTranslation) {
            this.targetTranslation = targetTranslation;
        }

        public Translation2d getTargetTranslation() {
            return targetTranslation;
        }
    }
    protected static String LogKey = "Superstructure/ShotCalculator";

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

    private static final InterpolatingTreeMap<Double, HoodShooterCalculation> shotDataMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            HoodShooterCalculation.interpolator
    );

    static {
        shotDataMap.put(1d, new HoodShooterCalculation(
                Rotation2d.fromRadians(0.437138669),
                10,
                1
        ));

        shotDataMap.put(1.5d, new HoodShooterCalculation(
                Rotation2d.fromRadians(0.611352924),
                10,
                1.05
        ));

        shotDataMap.put(2d, new HoodShooterCalculation(
                Rotation2d.fromRadians(0.75159462),
                10,
                1.1
        ));

        shotDataMap.put(2.5d, new HoodShooterCalculation(
                Rotation2d.fromRadians(0.86282925),
                10,
                1.15
        ));

        shotDataMap.put(3d, new HoodShooterCalculation(
                Rotation2d.fromRadians(0.951177756),
                10,
                1.2
        ));

        shotDataMap.put(3.5d, new HoodShooterCalculation(
                Rotation2d.fromRadians(1.022015817),
                10,
                1.25
        ));

        shotDataMap.put(4d, new HoodShooterCalculation(
                Rotation2d.fromRadians(1.079542321),
                10,
                1.3
        ));

        shotDataMap.put(4.5d, new HoodShooterCalculation(
                Rotation2d.fromRadians(1.126894805),
                10,
                1.35
        ));

        shotDataMap.put(5d, new HoodShooterCalculation(
                Rotation2d.fromRadians(1.166387399),
                10,
                1.4
        ));
    }

    public record ShotCalculation(
            Rotation2d desiredTurretRotation,
            HoodShooterCalculation hoodShooterCalculation
    ) {}

    public static ShotCalculation getShotCalculation(
            final Supplier<Pose2d> swervePoseSupplier,
            final Supplier<ChassisSpeeds> swerveChassisSpeedsSupplier,
            final Supplier<Translation2d> targetTranslationSupplier
    ) {
        return getShotCalculation(
                swervePoseSupplier.get(),
                swerveChassisSpeedsSupplier.get(),
                targetTranslationSupplier.get()
        );
    }

    //TODO:  logging
    private static ShotCalculation getShotCalculation(
            final Pose2d swervePose,
            final ChassisSpeeds swerveChassisSpeeds,
            final Translation2d target
    ) {
        final Pose2d turretPose = swervePose.transformBy(SimConstants.Turret.TURRET_TO_ROBOT_TRANSFORM);
        final double turretToTargetDistance = target.getDistance(turretPose.getTranslation());

        final double robotAngleRads = swervePose.getRotation().getRadians();
        final double turretVelocityX =
                swerveChassisSpeeds.vxMetersPerSecond
                    + swerveChassisSpeeds.omegaRadiansPerSecond
                        * (SimConstants.Turret.TURRET_TO_ROBOT_TRANSFORM.getY() * Math.cos(robotAngleRads)
                            - SimConstants.Turret.TURRET_TO_ROBOT_TRANSFORM.getX() * Math.sin(robotAngleRads));
        final double turretVelocityY =
                swerveChassisSpeeds.vyMetersPerSecond
                    + swerveChassisSpeeds.omegaRadiansPerSecond
                        * (SimConstants.Turret.TURRET_TO_ROBOT_TRANSFORM.getX() * Math.cos(robotAngleRads)
                            - SimConstants.Turret.TURRET_TO_ROBOT_TRANSFORM.getY() * Math.sin(robotAngleRads));

        final double timeOfFlight = shotDataMap.get(turretToTargetDistance)
                .shotTime;
        final double offsetX = turretVelocityX * timeOfFlight;
        final double offsetY = turretVelocityY * timeOfFlight;

        final Pose2d futurePose =
                new Pose2d(
                        turretPose.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                        turretPose.getRotation()
                );

        final double futureTurretToTargetDistance = target.getDistance(futurePose.getTranslation());

        final Rotation2d desiredTurretAngle = target.minus(futurePose.getTranslation()).getAngle().minus(futurePose.getRotation());


//        final Rotation2d differenceInAngle = target.minus(swervePose.getTranslation()).getAngle();
//
//        final Rotation2d desiredTurretRotation = differenceInAngle.minus(swervePose.getRotation());
//
//        final double distanceToTarget = target.getDistance(swervePose.getTranslation())

        final ShotCalculation shotCalculation = new ShotCalculation(
                desiredTurretAngle,
                shotDataMap.get(
                        futureTurretToTargetDistance
                )
        );
        Logger.recordOutput(LogKey + "/ShotCalculation", shotCalculation);

        return shotCalculation;
    }
}
