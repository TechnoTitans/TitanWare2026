package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.function.Function;
import java.util.function.Supplier;

public class MovingTOFShot {
    private static final double LookaheadSeconds = 0.025;
    private static final InterpolatingDoubleTreeMap TimeOfFlightMap = new InterpolatingDoubleTreeMap();
    static {
        TimeOfFlightMap.put(1.989, 1.2);
        TimeOfFlightMap.put(2.209, 1.15);
        TimeOfFlightMap.put(2.823, 1.18);
        TimeOfFlightMap.put(3.17, 1.13);
        TimeOfFlightMap.put(3.213, 1.01);
        TimeOfFlightMap.put(3.664, 1.19);
        TimeOfFlightMap.put(3.715, 1.08);
        TimeOfFlightMap.put(4.643, 1.21);
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

    public static ChassisSpeeds getTurretFieldSpeeds(
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

    public static ShotParameters getParameters(
            final Pose2d robotPose,
            final Translation2d turretTranslation,
            final ChassisSpeeds robotSpeeds,
            final Pose2d targetPose
    ) {
        final Pose2d offsetTurretPose = new Pose2d(turretTranslation, Rotation2d.kZero);
        final double distance = turretTranslation
                .getDistance(targetPose.getTranslation());
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

        final Transform2d turretOffset = offsetTurretPose.minus(lookaheadRobotPose);
        final ChassisSpeeds turretFieldVelocity = getTurretFieldSpeeds(lookaheadRobotPose, turretOffset, fieldSpeeds);

        double timeOfFlight;
        Pose2d futureTurretPose = offsetTurretPose;
        double futureDistance = distance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = TimeOfFlightMap.get(futureDistance);

            final Translation2d delta = new Translation2d(
                    turretFieldVelocity.vxMetersPerSecond * timeOfFlight,
                    turretFieldVelocity.vyMetersPerSecond * timeOfFlight
            );
            futureTurretPose = new Pose2d(
                    offsetTurretPose.getTranslation().plus(delta),
                    offsetTurretPose.getRotation()
            );
            futureDistance = futureTurretPose.getTranslation()
                    .getDistance(targetPose.getTranslation());
        }

        final Pose2d futureRobotPose = futureTurretPose.transformBy(turretOffset.inverse());
        return StaticShot.getParameters(
                futureRobotPose,
                futureTurretPose.getTranslation(),
                robotSpeeds,
                targetPose
        );
    }

    public static Supplier<ShotParameters> parametersSupplier(
            final Supplier<Pose2d> robotPoseSupplier,
            final Function<Pose2d, Translation2d> toTurretFn,
            final Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
            final Supplier<Pose2d> targetPoseSupplier
    ) {
        return () -> {
            final Pose2d robotPose = robotPoseSupplier.get();
            return MovingTOFShot.getParameters(
                    robotPose,
                    toTurretFn.apply(robotPose),
                    robotRelativeSpeedsSupplier.get(),
                    targetPoseSupplier.get()
            );
        };
    }
}
