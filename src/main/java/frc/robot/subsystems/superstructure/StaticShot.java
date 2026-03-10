package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import java.util.function.Function;
import java.util.function.Supplier;

public class StaticShot {
    public static final InterpolatingTreeMap<Double, ShotParameters.Shooter> ShotMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotParameters.Shooter::interpolate);
    static {
        ShotMap.put(1.7696, new ShotParameters.Shooter(30, 0));
        ShotMap.put(2.4459, new ShotParameters.Shooter(30.55, 0.0056));
        ShotMap.put(2.9222, new ShotParameters.Shooter(35, 0.0056));
        ShotMap.put(3.4454, new ShotParameters.Shooter(35, 0.0264));
        ShotMap.put(3.664, new ShotParameters.Shooter(35, 0.0444));
        ShotMap.put(3.7005, new ShotParameters.Shooter(35, 0.0537));
        ShotMap.put(3.973, new ShotParameters.Shooter(35, 0.05));
        ShotMap.put(4.9717, new ShotParameters.Shooter(40, 0.0583));
        ShotMap.put(5.43, new ShotParameters.Shooter(40, 0.0583));
    }

    public static Rotation2d angleToTarget(
            final Pose2d robotPose,
            final Translation2d turretTranslation,
            final Pose2d targetPose
    ) {
        return targetPose.getTranslation()
                .minus(turretTranslation)
                .getAngle()
                .minus(robotPose.getRotation());
    }

    public static ShotParameters getParameters(
            final Pose2d robotPose,
            final Translation2d turretTranslation,
            final ChassisSpeeds robotRelativeSpeeds,
            final Pose2d targetPose
    ) {
        return new ShotParameters(
                ShotMap.get(
                        turretTranslation
                                .getDistance(targetPose.getTranslation())
                ),
                angleToTarget(robotPose, turretTranslation, targetPose),
                Units.radiansToRotations(-robotRelativeSpeeds.omegaRadiansPerSecond)
        );
    }

    public static Supplier<ShotParameters> parametersSupplier(
            final Supplier<Pose2d> robotPoseSupplier,
            final Function<Pose2d, Translation2d> toTurretFn,
            final Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
            final Supplier<Pose2d> targetPoseSupplier
    ) {
        return () -> {
            final Pose2d robotPose = robotPoseSupplier.get();
            return StaticShot.getParameters(
                    robotPose,
                    toTurretFn.apply(robotPose),
                    robotRelativeSpeedsSupplier.get(),
                    targetPoseSupplier.get()
            );
        };
    }
}
