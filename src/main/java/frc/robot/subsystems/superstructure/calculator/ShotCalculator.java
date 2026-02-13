package frc.robot.subsystems.superstructure.calculator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.SimConstants;

import java.util.function.Supplier;

import static frc.robot.subsystems.superstructure.calculator.ShotCalculationStructs.FerryXBoundary;
import static frc.robot.subsystems.superstructure.calculator.ShotCalculationStructs.shotDataMap;

public class ShotCalculator {
    public enum Target {
        HUB,
        FERRYING
    }

    private static final double DelayTimeSec = 0.005;
    //TODO: Try to see if this is worth it
//    private static final LinearFilter turretFilter = LinearFilter.movingAverage(5);

    public static ShotCalculationStructs.ShotCalculation getShotCalculation(
            final Supplier<Pose2d> swervePoseSupplier,
            final Supplier<RobotCommands.ScoringMode> scoringModeSupplier,
            final Supplier<ChassisSpeeds> fieldRelativeSwerveSpeedsSupplier
    ) {
        final RobotCommands.ScoringMode scoringMode = scoringModeSupplier.get();

        return switch (scoringMode) {
            case Stationary -> getShotCalculation(swervePoseSupplier.get(), fieldRelativeSwerveSpeedsSupplier.get());
            case Moving -> getMovingShotCalculation(swervePoseSupplier.get(), fieldRelativeSwerveSpeedsSupplier.get());
        };
    }

    private static ShotCalculationStructs.ShotCalculation getShotCalculation(
            final Pose2d swervePose,
            final ChassisSpeeds swerveSpeeds
    ) {
        final Pose2d turretPose = swervePose.transformBy(SimConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D)
                .exp(new Twist2d(
                        swerveSpeeds.vxMetersPerSecond * DelayTimeSec,
                        swerveSpeeds.vyMetersPerSecond * DelayTimeSec,
                        swerveSpeeds.omegaRadiansPerSecond * DelayTimeSec
                ));

        final Pose2d calculationPose = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                ? turretPose : turretPose.relativeTo(FieldConstants.RED_ORIGIN);

        final Target target =
                calculationPose.getX()
                        > FerryXBoundary ? Target.FERRYING : Target.HUB;

        final Translation2d targetTranslation =
                switch (target) {
                    case HUB -> FieldConstants.getHubTarget();
                    case FERRYING -> FieldConstants.getFerryingTarget(calculationPose.getY());
                };

        final double turretToTargetDistance = targetTranslation.getDistance(turretPose.getTranslation());

        final Rotation2d desiredTurretAngle = targetTranslation.minus(turretPose.getTranslation()).getAngle().minus(
                turretPose.getRotation()
        );

        return new ShotCalculationStructs.ShotCalculation(
                desiredTurretAngle,
                shotDataMap.get(
                        turretToTargetDistance
                ),
                target
        );
    }
    //TODO: Needs to be implemented
    private static ShotCalculationStructs.ShotCalculation getMovingShotCalculation(
            final Pose2d swervePose,
            final ChassisSpeeds swerveSpeeds
    ) {
        final Pose2d turretPose = swervePose.transformBy(SimConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D)
                .exp(new Twist2d(
                        swerveSpeeds.vxMetersPerSecond * DelayTimeSec,
                        swerveSpeeds.vyMetersPerSecond * DelayTimeSec,
                        swerveSpeeds.omegaRadiansPerSecond * DelayTimeSec
                ));

        final Pose2d calculationPose = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                ? turretPose : turretPose.relativeTo(FieldConstants.RED_ORIGIN);

        final Target target =
                calculationPose.getX()
                        > FerryXBoundary ? Target.FERRYING : Target.HUB;

        final Translation2d targetTranslation =
                switch (target) {
                    case HUB -> FieldConstants.getHubTarget();
                    case FERRYING -> FieldConstants.getFerryingTarget(calculationPose.getY());
                };

        final double turretToTargetDistance = targetTranslation.getDistance(turretPose.getTranslation());

        final Rotation2d desiredTurretAngle = targetTranslation.minus(turretPose.getTranslation()).getAngle().minus(
                turretPose.getRotation()
        );

        return new ShotCalculationStructs.ShotCalculation(
                desiredTurretAngle,
                shotDataMap.get(
                        turretToTargetDistance
                ),
                target
        );
    }
}
