package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.geometry.AllianceFlipUtil;

import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.ShotCalculationData.FerryXBoundary;
import static frc.robot.subsystems.superstructure.ShotCalculationData.shotDataMap;

public class ShotCalculator {
    public enum Target {
        HUB(FieldConstants.Hub.hubCenterPoint),
        FERRYING(FieldConstants.ferryTarget);

        private final Translation2d targetTranslation;

        Target(final Translation2d targetTranslation) {
            this.targetTranslation = targetTranslation;
        }

        public Translation2d getTargetTranslation() {
            return targetTranslation;
        }
    }

    public static ShotCalculationData.ShotCalculation getShotCalculation(
            final Supplier<Pose2d> swervePoseSupplier,
            final Supplier<RobotCommands.ScoringMode> scoringModeSupplier
    ) {
        final RobotCommands.ScoringMode scoringMode = scoringModeSupplier.get();

        return switch (scoringMode) {
            case Stationary, Turret_Off -> getShotCalculation(swervePoseSupplier.get());
            case Moving -> getMovingShotCalculation(swervePoseSupplier.get());
        };
    }

    private static ShotCalculationData.ShotCalculation getShotCalculation(
            final Pose2d swervePose
    ) {
        final Pose2d turretPose = swervePose.transformBy(SimConstants.Turret.TURRET_TO_ROBOT_TRANSFORM);

        Target target = turretPose.getX() > FerryXBoundary ? Target.FERRYING : Target.HUB;
        final Translation2d targetTranslation = AllianceFlipUtil.apply(target.getTargetTranslation());

        final double turretToTargetDistance = targetTranslation.getDistance(turretPose.getTranslation());

        final Rotation2d desiredTurretAngle = targetTranslation.minus(turretPose.getTranslation()).getAngle().minus(
                turretPose.getRotation()
        );

        return new ShotCalculationData.ShotCalculation(
                desiredTurretAngle,
                shotDataMap.get(
                        turretToTargetDistance
                ),
                target
        );
    }
    //TODO: Needs to be implemented
    private static ShotCalculationData.ShotCalculation getMovingShotCalculation(
            final Pose2d swervePose
    ) {
        final Pose2d turretPose = swervePose.transformBy(SimConstants.Turret.TURRET_TO_ROBOT_TRANSFORM);

        Target target = turretPose.getX() > FerryXBoundary ? Target.FERRYING : Target.HUB;
        final Translation2d targetTranslation = AllianceFlipUtil.apply(target.getTargetTranslation());

        final double turretToTargetDistance = targetTranslation.getDistance(turretPose.getTranslation());

        final Rotation2d desiredTurretAngle = targetTranslation.minus(turretPose.getTranslation()).getAngle().minus(
                turretPose.getRotation()
        );

        return new ShotCalculationData.ShotCalculation(
                desiredTurretAngle,
                shotDataMap.get(
                        turretToTargetDistance
                ),
                target
        );
    }
}
