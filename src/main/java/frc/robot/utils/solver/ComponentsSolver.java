package frc.robot.utils.solver;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ComponentsSolver {
    final Supplier<Rotation2d> turretRotationSupplier;
    final Supplier<Rotation2d> hoodRotationSupplier;
    final Supplier<Rotation2d> intakeSlideRotationSupplier;
    final DoubleSupplier climbExtensionMetersSupplier;

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
        final Pose3d turretPose = new Pose3d(
                SimConstants.Turret.ORIGIN,
                new Rotation3d(turretRotationSupplier.get().plus(Rotation2d.kCW_90deg))
        );

        final Pose3d hoodPose = turretPose.transformBy(
                new Transform3d(
                        SimConstants.Hood.TURRET_TO_HOOD_TRANSLATION,
                        new Rotation3d(
                                hoodRotationSupplier.get().unaryMinus().plus(SimConstants.Hood.ZEROED_POSITION_TO_HORIZONTAL).getRadians(),
                                0,
                                0
                        )
                )
        );

        return new Pose3d[] {turretPose, hoodPose};
    }

    private Pose3d[] getIntakeHopperPoses() {
        final double dt = intakeSlideRotationSupplier.get().getRotations()
                        / (HardwareConstants.INTAKE_SLIDE.upperLimitRots() - HardwareConstants.INTAKE_SLIDE.lowerLimitRots());

        final Pose3d intakePose = SimConstants.IntakeSlide.RETRACTED_POSE
                .interpolate(SimConstants.IntakeSlide.EXTENDED_POSE, dt);

        final Pose3d hopperPose = SimConstants.Hopper.RETRACTED_POSE
                .interpolate(SimConstants.Hopper.EXTENDED_POSE, dt);

        return new Pose3d[] {intakePose, hopperPose};
    }

    private Pose3d[] getClimbPoses() {
        final double extensionMeters = climbExtensionMetersSupplier.getAsDouble();

        final double stage1ExtensionMeters = Math.min(extensionMeters, SimConstants.Climb.STAGE_1_MAX_EXTENSION);
        final Pose3d stage1Pose = SimConstants.Climb.ORIGIN.transformBy(
                new Transform3d(new Translation3d(stage1ExtensionMeters, Rotation3d.kZero), SimConstants.Climb.ANGLE_FROM_HORIZONTAL)
        );

        final double stage2ExtensionMeters = Math.min(extensionMeters - stage1ExtensionMeters, SimConstants.Climb.STAGE_2_MAX_EXTENSION);
        final Pose3d stage2Pose = SimConstants.Climb.ORIGIN.transformBy(
                new Transform3d(new Translation3d(stage2ExtensionMeters, Rotation3d.kZero), SimConstants.Climb.ANGLE_FROM_HORIZONTAL)
        );


        return new Pose3d[] {stage1Pose, stage2Pose};
    }
}
