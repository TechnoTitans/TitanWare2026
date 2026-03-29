package frc.robot.subsystems.superstructure.calculation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.constants.PoseConstants;

import java.util.function.Supplier;

public class StaticShot {
    private static final InterpolatingTreeMap<Double, ShotCalculation.ShooterCalculation> ShotMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotCalculation.ShooterCalculation::interpolate);
    static {
        if (Constants.CURRENT_MODE != Constants.RobotMode.SIM) {
            ShotMap.put(1.30, new ShotCalculation.ShooterCalculation(28, 0));
            ShotMap.put(2.07, new ShotCalculation.ShooterCalculation(32, 0.005));
            ShotMap.put(2.42, new ShotCalculation.ShooterCalculation(32, 0.008));
            ShotMap.put(2.67, new ShotCalculation.ShooterCalculation(33, 0.01));
            ShotMap.put(3.07, new ShotCalculation.ShooterCalculation(35, 0.012));
            ShotMap.put(3.60, new ShotCalculation.ShooterCalculation(36, 0.014));
            ShotMap.put(4.54, new ShotCalculation.ShooterCalculation(40, 0.021));
            ShotMap.put(5.18, new ShotCalculation.ShooterCalculation(43, 0.025));
            ShotMap.put(5.84, new ShotCalculation.ShooterCalculation(44, 0.027));
            ShotMap.put(6.16, new ShotCalculation.ShooterCalculation(44, 0.041));
        } else {
            ShotMap.put(1.3, new ShotCalculation.ShooterCalculation(21, 0));
            ShotMap.put(1.5, new ShotCalculation.ShooterCalculation(21, 0.008));
            ShotMap.put(2.0, new ShotCalculation.ShooterCalculation(21, 0.0175));
            ShotMap.put(2.5, new ShotCalculation.ShooterCalculation(21, 0.031));
            ShotMap.put(3.0, new ShotCalculation.ShooterCalculation(22.5, 0.034));
            ShotMap.put(3.5, new ShotCalculation.ShooterCalculation(23, 0.045));
            ShotMap.put(4.0, new ShotCalculation.ShooterCalculation(24, 0.0475));
            ShotMap.put(4.5, new ShotCalculation.ShooterCalculation(25.5, 0.05));
            ShotMap.put(5.0, new ShotCalculation.ShooterCalculation(26.5, 0.0525));
            ShotMap.put(5.5, new ShotCalculation.ShooterCalculation(27.5, 0.055));
            ShotMap.put(6.0, new ShotCalculation.ShooterCalculation(28.75, 0.0565));
        }
    }

    public static Supplier<ShotCalculation> getShotCalculationSupplier(
            final Supplier<Pose2d> robotPoseSupplier,
            final Supplier<ChassisSpeeds> robotRelativeSpeeds,
            final Supplier<Pose2d> targetPoseSupplier
    ) {
        return () -> getShotCalculation(
                robotPoseSupplier.get(),
                robotRelativeSpeeds.get(),
                targetPoseSupplier.get()
        );
    }

    public static ShotCalculation getShotCalculation(
            final Pose2d robotPose,
            final ChassisSpeeds robotSpeeds,
            final Pose2d targetPose
    ) {
        final Pose2d turretPose = robotPose.transformBy(PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D);

        final Translation2d targetTranslation = targetPose.getTranslation();
        final double turretToTargetDistance = targetTranslation.getDistance(turretPose.getTranslation());
        final Rotation2d desiredTurretAngle = targetTranslation.minus(turretPose.getTranslation())
                .getAngle()
                .minus(robotPose.getRotation());

        return new ShotCalculation(
                desiredTurretAngle.getRotations(),
                Units.radiansToRotations(-robotSpeeds.omegaRadiansPerSecond),
                ShotMap.get(turretToTargetDistance)
        );
    }
}
