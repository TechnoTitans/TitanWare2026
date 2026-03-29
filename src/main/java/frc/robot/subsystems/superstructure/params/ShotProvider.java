package frc.robot.subsystems.superstructure.params;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Supplier;

@SuppressWarnings("unused")
public interface ShotProvider<T extends ShotProvider.Kind> {
    interface Kind {
        class Static implements Kind {}
        class Moving implements Kind {}
    }

    ShotParameters getParameters(
            final Pose2d robotPose,
            final Transform2d robotToTurret,
            final ChassisSpeeds robotSpeeds,
            final Pose2d targetPose
    );

    default Supplier<ShotParameters> parametersSupplier(
            final Supplier<Pose2d> robotPoseSupplier,
            final Supplier<Transform2d> robotToTurretSupplier,
            final Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
            final Supplier<Pose2d> targetPoseSupplier
    ) {
        return () -> {
            final Pose2d robotPose = robotPoseSupplier.get();
            return getParameters(
                    robotPose,
                    robotToTurretSupplier.get(),
                    robotRelativeSpeedsSupplier.get(),
                    targetPoseSupplier.get()
            );
        };
    }
}
