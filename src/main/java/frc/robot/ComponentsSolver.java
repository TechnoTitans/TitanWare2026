package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.PoseConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

@SuppressWarnings("ClassCanBeRecord")
public class ComponentsSolver {
    private final Supplier<Rotation2d> turretRotationSupplier;
    private final Supplier<Rotation2d> hoodRotationSupplier;
    private final Supplier<Rotation2d> intakeSlideRotationSupplier;

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
                PoseConstants.Turret.ORIGIN,
                new Rotation3d(turretRotationSupplier.get())
        );

        final Pose3d hoodPose = turretPose
                .plus(new Transform3d(
                        PoseConstants.Hood.TURRET_TO_HOOD_TRANSLATION,
                        new Rotation3d(0, hoodRotationSupplier.get().getRadians(), 0)
                ));

        return new Pose3d[] {
                turretPose,
                hoodPose
        };
    }

    private Pose3d[] getIntakeHopperPoses() {
        final double extensionRatio = intakeSlideRotationSupplier.get().getRotations()
                / (HardwareConstants.INTAKE_SLIDE.forwardLimitRots() - HardwareConstants.INTAKE_SLIDE.reverseLimitRots());

        final Pose3d intakePose = PoseConstants.IntakeSlide.RETRACTED_POSE
                .interpolate(PoseConstants.IntakeSlide.EXTENDED_POSE, extensionRatio);

        final Pose3d hopperPose = PoseConstants.Hopper.RETRACTED_POSE
                .interpolate(PoseConstants.Hopper.EXTENDED_POSE, extensionRatio);

        return new Pose3d[]{intakePose, hopperPose};
    }
}