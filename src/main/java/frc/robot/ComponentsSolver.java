package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.SimConstants;
import frc.robot.utils.subsystems.VirtualSubsystem;

import java.util.function.Supplier;

public class ComponentsSolver extends VirtualSubsystem {
    public static Pose3d[] getSuperstructurePoses(
            final Supplier<Rotation2d> turretRotationSupplier,
            final Supplier<Rotation2d> hoodRotationSupplier
    ) {
        final Pose3d turretPose = new Pose3d(
                SimConstants.Turret.ORIGIN_OFFSET,
                new Rotation3d(turretRotationSupplier.get())
        );

        final Pose3d hoodPose = turretPose
                .plus(new Transform3d(
                        SimConstants.Hood.TURRET_OFFSET,
                        new Rotation3d(0, hoodRotationSupplier.get().getRadians(), 0)
                ));

        return new Pose3d[] {
                turretPose,
                hoodPose
        };
    }

    public static Pose3d[] getIntakeHopperPoses(final Supplier<Rotation2d> intakeSlideRotationSupplier) {
        final Pose3d slideExtended = SimConstants.IntakeSlide.EXTENDED_POSE;
        final Pose3d slideRetracted = SimConstants.IntakeSlide.RETRACTED_POSE;

        final Pose3d hopperExtended = SimConstants.Hopper.EXTENDED_POSE;
        final Pose3d hopperRetracted = SimConstants.Hopper.RETRACTED_POSE;

        final double extensionMeters = intakeSlideRotationSupplier.get().getRotations()
                * SimConstants.IntakeSlide.SlideRotationsToLinearDistanceMetersRatio;
        final double totalExtensionDistance = slideExtended.getTranslation()
                .getDistance(slideRetracted.getTranslation());
        final double extensionRatio = extensionMeters / totalExtensionDistance;

        return new Pose3d[] {
                slideRetracted.interpolate(slideExtended, extensionRatio),
                hopperRetracted.interpolate(hopperExtended, extensionRatio)
        };
    }
}
