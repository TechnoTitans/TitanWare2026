package frc.robot.utils.solver;

import edu.wpi.first.math.geometry.*;
import frc.robot.constants.SimConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ComponentsSolver {
    final Supplier<Rotation2d> turretRotationSupplier;
    final Supplier<Rotation2d> hoodRotationSupplier;
//    final Supplier<Rotation2d> intakeRotationSupplier;

    public ComponentsSolver(
            final Supplier<Rotation2d> turretRotationSupplier,
            final Supplier<Rotation2d> hoodRotationSupplier
//            final Supplier<Rotation2d> intakeRotationSupplier
    ) {
        this.turretRotationSupplier = turretRotationSupplier;
        this.hoodRotationSupplier = hoodRotationSupplier;
//        this.intakeRotationSupplier = intakeRotationSupplier;
    }

    public void periodic() {
        final Pose3d[] superstructurePoses = getSuperstructurePoses();
        final Pose3d[] intakePoses = getIntakePoses();

        Logger.recordOutput(
                "Components",
                superstructurePoses[0],
                superstructurePoses[1],
                intakePoses[0],
                intakePoses[1]
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

    private Pose3d[] getIntakePoses() {
        return new Pose3d[]{};
    }
}
