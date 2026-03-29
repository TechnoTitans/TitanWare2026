package frc.robot.subsystems.superstructure.params;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class MovingNewtonsTOFShot implements ShotProvider<ShotProvider.Kind.Moving> {
    private static final double Epsilon = 0.001;
    private static final double VelocityTolerance = 0.005;
    private static final double LookaheadSeconds = 0.01;

    // TODO: need to move this out somewhere so it's not duplicated in MovingTOFShot too
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
        // I referenced
        // https://www.chiefdelphi.com/t/4322-clockwork-2026-build-thread-open-alliance/511196/49?u=harryxchen

        // this doesn't work that well if you try to do some twirly whirlies
        // you should notice that it kinda starts missing balls far/too high for some reason

        // for the turretOffset and distance this currently matches the #2 behavior in MovingTOFShot
        // I have also commented out what I think the #1 version would look like,
        // I think I tried it, but it didn't really help

        // you want to look through it, maybe give it a try and see if you can make it better?

        final Pose2d lookaheadRobotPose = robotPose.exp(new Twist2d(
                robotSpeeds.vxMetersPerSecond * LookaheadSeconds,
                robotSpeeds.vyMetersPerSecond * LookaheadSeconds,
                robotSpeeds.omegaRadiansPerSecond * LookaheadSeconds
        ));
//        final Transform2d turretOffset = offsetTurretPose.minus(lookaheadRobotPose);
        final Pose2d lookaheadTurretPose = lookaheadRobotPose.transformBy(robotToTurret);

        final Translation2d toGoal = targetPose.getTranslation()
                .minus(lookaheadTurretPose.getTranslation());
        final double distance = toGoal.getNorm();
//        final double distance = targetPose.getTranslation().getDistance(turretTranslation);
        final Translation2d targetDirection = toGoal.div(distance);

        final double initialTimeOfFlight = ShotParameters.getTimeOfFlight(distance);
        final double initialVelocity = distance / initialTimeOfFlight;

        final Rotation2d robotAngle = lookaheadRobotPose.getRotation();
        final ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotAngle);
        final ChassisSpeeds turretFieldSpeeds =
                getTurretFieldSpeeds(lookaheadRobotPose, robotToTurret, fieldSpeeds);

        final Translation2d targetVelocity = targetDirection.times(initialVelocity);
        final Translation2d turretVelocity = new Translation2d(
                turretFieldSpeeds.vxMetersPerSecond,
                turretFieldSpeeds.vyMetersPerSecond
        );
        final Translation2d shotVelocity = targetVelocity.minus(turretVelocity);

        final Rotation2d turretFieldAngle = shotVelocity.getAngle();
        final double requiredVelocity = shotVelocity.getNorm();

        double futureDistance = distance;
        double timeOfFlight;
        double futureVelocity = initialVelocity;

        for (int i = 0; i < 20; i++) {
            final double lowVel = (futureDistance - Epsilon)
                    / ShotParameters.getTimeOfFlight(futureDistance - Epsilon);
            final double highVel = (futureDistance + Epsilon)
                    / ShotParameters.getTimeOfFlight(futureDistance + Epsilon);
            final double velPrime = (highVel - lowVel) / (2 * Epsilon);

            futureDistance -= (futureVelocity - requiredVelocity) / velPrime;
            timeOfFlight = ShotParameters.getTimeOfFlight(futureDistance);
            futureVelocity = futureDistance / timeOfFlight;

            if (Math.abs(futureVelocity - requiredVelocity) <= VelocityTolerance) {
                break;
            }
        }

        return new ShotParameters(
                ShotParameters.getShot(futureDistance),
                turretFieldAngle
                        .minus(robotAngle),
                Units.radiansToRotations(-robotSpeeds.omegaRadiansPerSecond)
        );
    }
}
