package frc.robot.utils.solver;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ComponentsSolver {
    final Supplier<Rotation2d> turretRotationSupplier;
    final Supplier<Rotation2d> hoodRotationSupplier;
    final Supplier<Rotation2d> intakeSlideRotationSupplier;

    public ComponentsSolver(
            final Supplier<Rotation2d> turretRotationSupplier,
            final Supplier<Rotation2d> hoodRotationSupplier,
            final Supplier<Rotation2d> intakeSlideRotationSupplier
    ) {
        this.turretRotationSupplier = turretRotationSupplier;
        this.hoodRotationSupplier = hoodRotationSupplier;
        this.intakeSlideRotationSupplier = intakeSlideRotationSupplier;
    }

    public void periodic() {
        final Pose3d[] superstructurePoses = getSuperstructurePoses();
        final Pose3d[] intakeHopperPoses = getIntakeHopperPoses();

        Logger.recordOutput(
                "Components",
                superstructurePoses[0],
                superstructurePoses[1],
                intakeHopperPoses[0],
                intakeHopperPoses[1]
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
//                                hoodRotationSupplier.get().minus(SimConstants.Hood.ZEROED_POSITION_TO_HORIZONTAL)
//                                        .getRadians(),
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
}
