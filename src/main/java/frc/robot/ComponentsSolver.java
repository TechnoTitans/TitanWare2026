package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.PoseConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

//TODO: Fix with new cad models and positions
@SuppressWarnings("ClassCanBeRecord")
public class ComponentsSolver {
    private final Supplier<Rotation2d> turretRotationSupplier;
    private final Supplier<Rotation2d> hoodRotationSupplier;
    private final Supplier<Rotation2d> intakeSlideRotationSupplier;
    private final DoubleSupplier climbExtensionMetersSupplier;

    public ComponentsSolver(
            final Supplier<Rotation2d> turretRotationSupplier,
            final Supplier<Rotation2d> hoodRotationSupplier,
            final Supplier<Rotation2d> intakeSlideRotationSupplier,
            final DoubleSupplier climbExtensionMetersSupplier
    ) {
        this.turretRotationSupplier = turretRotationSupplier;
        this.hoodRotationSupplier = hoodRotationSupplier;
        this.intakeSlideRotationSupplier = intakeSlideRotationSupplier;
        this.climbExtensionMetersSupplier = climbExtensionMetersSupplier;
    }

    public void periodic() {
        final Pose3d[] superstructurePoses = getSuperstructurePoses();
        final Pose3d[] intakeHopperPoses = getIntakeHopperPoses();
        final Pose3d[] climbPoses = getClimbPoses();

        Logger.recordOutput(
                "Components",
                superstructurePoses[0],
                superstructurePoses[1],
                intakeHopperPoses[0],
                intakeHopperPoses[1],
                climbPoses[0],
                climbPoses[1]
        );
    }

    private Pose3d[] getSuperstructurePoses() {
        //TODO: Turret zero is different now
        final Pose3d turretPose = new Pose3d(
                PoseConstants.Turret.ORIGIN,
                new Rotation3d(turretRotationSupplier.get())
                        .rotateBy(PoseConstants.Turret.TURRET_ZERO_OFFSET)
        );

        final Pose3d hoodPose = turretPose.transformBy(
                new Transform3d(
                        PoseConstants.Hood.TURRET_TO_HOOD_TRANSLATION,
                        new Rotation3d(
                                hoodRotationSupplier.get()
                                        .unaryMinus()
                                        .plus(PoseConstants.Hood.ZEROED_POSITION_TO_HORIZONTAL).getRadians(),
                                0,
                                0
                        )
                )
        );

        return new Pose3d[]{turretPose, hoodPose};
    }

    private Pose3d[] getIntakeHopperPoses() {
        final double extensionRatio = intakeSlideRotationSupplier.get().getRotations()
                / (HardwareConstants.INTAKE_SLIDE.upperLimitRots() - HardwareConstants.INTAKE_SLIDE.lowerLimitRots());

        final Pose3d intakePose = PoseConstants.IntakeSlide.RETRACTED_POSE
                .interpolate(PoseConstants.IntakeSlide.EXTENDED_POSE, extensionRatio);

        final Pose3d hopperPose = PoseConstants.Hopper.RETRACTED_POSE
                .interpolate(PoseConstants.Hopper.EXTENDED_POSE, extensionRatio);

        return new Pose3d[]{intakePose, hopperPose};
    }

    private Pose3d[] getClimbPoses() {
        final double extensionMeters = climbExtensionMetersSupplier.getAsDouble();

        final double stage1ExtensionMeters = Math.min(extensionMeters, PoseConstants.Climb.STAGE_1_MAX_EXTENSION);

        final Pose3d stage1Pose = new Pose3d(PoseConstants.Climb.ORIGIN,
                PoseConstants.Climb.ANGLE_FROM_HORIZONTAL
        ).transformBy(
                new Transform3d(
                        0,
                        0,
                        stage1ExtensionMeters,
                        Rotation3d.kZero
                )
        );

        final double stage2ExtensionMeters =
                Math.min(extensionMeters - stage1ExtensionMeters, PoseConstants.Climb.STAGE_2_MAX_EXTENSION);
        final Pose3d stage2Pose = stage1Pose.transformBy(
                new Transform3d(
                        0,
                        0,
                        stage2ExtensionMeters,
                        Rotation3d.kZero
                )
        );

        return new Pose3d[]{stage1Pose, stage2Pose};
    }
}
