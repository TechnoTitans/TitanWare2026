package frc.robot.subsystems.superstructure.params;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.ShootCommands;

public class StaticShot implements ShotProvider<ShotProvider.Kind.Static> {
    public static Rotation2d angleToTarget(
            final Pose2d robotPose,
            final Translation2d turretTranslation,
            final Pose2d targetPose
    ) {
        return targetPose.getTranslation()
                .minus(turretTranslation)
                .getAngle()
                .minus(robotPose.getRotation());
    }

    @Override
    public ShotParameters getParameters(
            final Pose2d robotPose,
            final Transform2d robotToTurret,
            final ChassisSpeeds robotRelativeSpeeds,
            final ShootCommands.Target target,
            final Pose2d targetPose
    ) {
        final Translation2d turretTranslation = robotPose
                .transformBy(robotToTurret)
                .getTranslation();
        return new ShotParameters(
                ShotParameters.getShot(
                        target,
                        turretTranslation
                                .getDistance(targetPose.getTranslation())
                ),
                angleToTarget(robotPose, turretTranslation, targetPose),
                Units.radiansToRotations(-robotRelativeSpeeds.omegaRadiansPerSecond)
        );
    }
}
