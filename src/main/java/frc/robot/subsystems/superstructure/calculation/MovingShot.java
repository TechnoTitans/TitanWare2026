package frc.robot.subsystems.superstructure.calculation;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Constants;
import frc.robot.constants.PoseConstants;

import java.util.function.Supplier;

public class MovingShot {
    private static final InterpolatingDoubleTreeMap TimeOfFlightMap = new InterpolatingDoubleTreeMap();
    private static final double LookaheadSeconds = 0.025;

    static {
        if (Constants.CURRENT_MODE != Constants.RobotMode.SIM) {
            TimeOfFlightMap.put(1.989, 1.2);
            TimeOfFlightMap.put(2.209, 1.3);
            TimeOfFlightMap.put(2.823, 1.4);
            TimeOfFlightMap.put(3.17, 1.5);
            TimeOfFlightMap.put(3.664d, 1.6);
            TimeOfFlightMap.put(4.643, 1.7);
        } else {
            TimeOfFlightMap.put(1.208, 1.163);
            TimeOfFlightMap.put(1.418, 1.06);
            TimeOfFlightMap.put(2.17, 1.0);
            TimeOfFlightMap.put(2.72, 1.02);
            TimeOfFlightMap.put(3.51, 1.06);
            TimeOfFlightMap.put(4.63, 1.2);
            TimeOfFlightMap.put(4.9, 1.24);
            TimeOfFlightMap.put(5.73, 1.31);
        }
    }

    public static Supplier<ShotCalculation> getShotCalculationSupplier(
            final Supplier<Pose2d> robotPoseSupplier,
            final Supplier<ChassisSpeeds> robotSpeedsSupplier,
            final Supplier<Pose2d> targetPoseSupplier
    ) {
        return () -> getShotCalculation(
                robotPoseSupplier.get(),
                robotSpeedsSupplier.get(),
                targetPoseSupplier.get()
        );
    }

    public static ShotCalculation getShotCalculation(
            final Pose2d robotPose,
            final ChassisSpeeds robotSpeeds,
            final Pose2d targetPose
    ) {
        final Pose2d turretPose = new Pose2d(
                robotPose.transformBy(PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D).getTranslation(),
                Rotation2d.kZero
        );

        final Translation2d targetTranslation = targetPose.getTranslation();
        final double turretToTargetDistance = targetTranslation.getDistance(turretPose.getTranslation());

        final Pose2d lookaheadRobotPose = robotPose.exp(new Twist2d(
                robotSpeeds.vxMetersPerSecond * LookaheadSeconds,
                robotSpeeds.vyMetersPerSecond * LookaheadSeconds,
                robotSpeeds.omegaRadiansPerSecond * LookaheadSeconds
        ));

        final Rotation2d robotAngle = lookaheadRobotPose.getRotation();
        final ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotSpeeds,
                robotAngle
        );

        final Transform2d turretOffset = turretPose.minus(lookaheadRobotPose);
        final ChassisSpeeds turretFieldVelocity = getTurretFieldSpeeds(lookaheadRobotPose, turretOffset, fieldSpeeds);

        double timeOfFlight;
        Pose2d futureTurretPose = turretPose;
        double futureDistance = turretToTargetDistance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = TimeOfFlightMap.get(futureDistance);

            final Translation2d delta = new Translation2d(
                    turretFieldVelocity.vxMetersPerSecond * timeOfFlight,
                    turretFieldVelocity.vyMetersPerSecond * timeOfFlight
            );
            futureTurretPose = new Pose2d(
                    turretPose.getTranslation().plus(delta),
                    turretPose.getRotation()
            );
            futureDistance = futureTurretPose
                    .getTranslation()
                    .getDistance(targetTranslation);
        }

        final Pose2d futureRobotPose = futureTurretPose.transformBy(turretOffset.inverse());

        return StaticShot.getShotCalculation(
                futureRobotPose,
                robotSpeeds,
                targetPose
        );
    }

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

    private static ChassisSpeeds getTurretFieldSpeeds(
            final Pose2d robotPose,
            final Transform2d turretOffset,
            final ChassisSpeeds fieldRelativeSpeeds
    ) {
        final double omega = fieldRelativeSpeeds.omegaRadiansPerSecond;
        final Translation2d rField = turretOffset
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
