package frc.robot.subsystems.superstructure.params;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.utils.control.DeltaTime;

public final class MovingTOFShot implements ShotProvider<ShotProvider.Kind.Moving> {
    private static final double LookaheadSeconds = 0.025;

    private final DeltaTime deltaTime = new DeltaTime();
    private final LinearFilter turretOmegaFilter =
            LinearFilter.movingAverage((int)(0.1 / Constants.LOOP_PERIOD_SECONDS));
    private double lastTurretAngleRads = 0;

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

    @Override
    public ShotParameters getParameters(
            final Pose2d robotPose,
            final Transform2d robotToTurret,
            final ChassisSpeeds robotSpeeds,
            final Pose2d targetPose
    ) {
        // the way the code is now is the version that I think is more correct
        // and the commented out code is an old version that seemed to work better in sim
        //  better in sim (also what we had before):
        //      1. turretOffset is calculated from the transform between the current turret translation
        //          and the lookahead robot pose, which is NOT actually the turretOffset,
        //          since the former isn't looked ahead
        //      2. distance is calculated between the current turret translation and the target pose,
        //          as opposed to from a lookahead turret translation
        //  more correct (imo)
        //      1. turretOffset from current robot to current turret pose
        //      2. distance is between lookahead turret and target

//        final Pose2d turretPose = robotPose.transformBy(robotToTurret);
        final Pose2d lookaheadRobotPose = robotPose.exp(new Twist2d(
                robotSpeeds.vxMetersPerSecond * LookaheadSeconds,
                robotSpeeds.vyMetersPerSecond * LookaheadSeconds,
                robotSpeeds.omegaRadiansPerSecond * LookaheadSeconds
        ));
//        final Transform2d turretOffset = offsetTurretPose.minus(lookaheadRobotPose);
        final Pose2d lookaheadTurretPose = lookaheadRobotPose.transformBy(robotToTurret);
        final double distance = lookaheadTurretPose.getTranslation()
                .getDistance(targetPose.getTranslation());
//        final double distance = turretTranslation
//                .getDistance(targetPose.getTranslation());

        final Rotation2d robotAngle = lookaheadRobotPose.getRotation();
        final ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotAngle);
        final ChassisSpeeds turretFieldVelocity =
                getTurretFieldSpeeds(lookaheadRobotPose, robotToTurret, fieldSpeeds);

        double timeOfFlight;
        Pose2d futureTurretPose = lookaheadTurretPose;
        double futureDistance = distance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = ShotParameters.getTimeOfFlight(futureDistance);

            final Translation2d delta = new Translation2d(
                    turretFieldVelocity.vxMetersPerSecond * timeOfFlight,
                    turretFieldVelocity.vyMetersPerSecond * timeOfFlight
            );
            futureTurretPose = new Pose2d(
                    lookaheadTurretPose.getTranslation().plus(delta),
                    lookaheadTurretPose.getRotation()
            );
            futureDistance = futureTurretPose.getTranslation()
                    .getDistance(targetPose.getTranslation());
        }

        final double dt = deltaTime.get();
        final Pose2d futureRobotPose = futureTurretPose.transformBy(robotToTurret.inverse());
        final Rotation2d turretAngle =
                StaticShot.angleToTarget(futureRobotPose, futureTurretPose.getTranslation(), targetPose);
        final double turretAngleRads = turretAngle.getRadians();
        final double turretOmegaRadsPerSec = turretOmegaFilter.calculate(
                MathUtil.angleModulus(turretAngleRads - lastTurretAngleRads)
                        / dt
        );
        lastTurretAngleRads = turretAngleRads;

        return new ShotParameters(
                ShotParameters.getShot(futureDistance),
                turretAngle,
                Units.radiansToRotations(turretOmegaRadsPerSec)
        );
    }
}
