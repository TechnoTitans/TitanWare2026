package frc.robot.subsystems.superstructure.params;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MovingUtils {
    public static ChassisSpeeds getTurretFieldSpeeds(
            final Pose2d robotPose,
            final Translation2d turretTranslation,
            final ChassisSpeeds fieldRelativeSpeeds
    ) {
        return getTurretFieldSpeeds(
                robotPose,
                new Transform2d(robotPose, new Pose2d(turretTranslation, Rotation2d.kZero)),
                fieldRelativeSpeeds
        );
    }

    public static ChassisSpeeds getTurretFieldSpeeds(
            final Pose2d robotPose,
            final Transform2d robotToTurret,
            final ChassisSpeeds fieldRelativeSpeeds
    ) {
        final double omega = fieldRelativeSpeeds.omegaRadiansPerSecond;
        final Translation2d rField = robotToTurret
                .getTranslation()
                .rotateBy(robotPose.getRotation());

        final double tangentVx = -omega * rField.getY();
        final double tangentVy = omega * rField.getX();

        return new ChassisSpeeds(
                fieldRelativeSpeeds.vxMetersPerSecond + tangentVx,
                fieldRelativeSpeeds.vyMetersPerSecond + tangentVy,
                omega
        );
    }
}