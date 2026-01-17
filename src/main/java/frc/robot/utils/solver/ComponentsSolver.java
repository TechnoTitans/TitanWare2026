package frc.robot.utils.solver;

import edu.wpi.first.math.geometry.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ComponentsSolver {
    final Supplier<Rotation2d> turretRotationSupplier;
    final Supplier<Rotation2d> hoodRotationSupplier;
    final Supplier<Rotation2d> intakeSliderRotationSupplier;

    public ComponentsSolver(
            final Supplier<Rotation2d> turretRotationSupplier,
            final Supplier<Rotation2d> hoodRotationSupplier,
            final Supplier<Rotation2d> intakeSliderRotationSupplier
    ) {
        this.turretRotationSupplier = turretRotationSupplier;
        this.hoodRotationSupplier = hoodRotationSupplier;
        this.intakeSliderRotationSupplier = intakeSliderRotationSupplier;
    }

    public void periodic() {
        final Pose3d[] superstructurePoses = getSuperstructurePoses();
        final Pose3d intakePose = getIntakePose();

        Logger.recordOutput(
                "Components",
                superstructurePoses[0],
                superstructurePoses[1],
                intakePose
        );
    }

    private Pose3d[] getSuperstructurePoses() {
        final Pose3d turretPose = new Pose3d(
                SimConstants.Turret.ORIGIN,
                new Rotation3d(turretRotationSupplier.get())
        );

        final Pose3d hoodPose = turretPose.transformBy(
                new Transform3d(
                        SimConstants.Hood.TURRET_TO_HOOD_TRANSLATION,
                        new Rotation3d(
                                0,
                                hoodRotationSupplier.get().minus(SimConstants.Hood.ZEROED_POSITION_TO_HORIZONTAL)
                                        .getRadians(),
                                0
                        )
                )
        );

        return new Pose3d[] {turretPose, hoodPose};
    }

    private Pose3d getIntakePose() {
        final double dt = intakeSliderRotationSupplier.get().getRotations()
                        / (HardwareConstants.INTAKE.upperLimitRots() - HardwareConstants.INTAKE.lowerLimitRots());
        final Pose3d intakePose = SimConstants.IntakeSlider.RETRACTED_POSE
                .interpolate(SimConstants.IntakeSlider.EXTENDED_POSE, dt);

        return intakePose;
    }
}
