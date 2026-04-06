package frc.robot.subsystems.superstructure.params;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.ShootCommands;
import frc.robot.constants.Constants;
import frc.robot.utils.control.DeltaTime;

public final class MovingTOFShot implements ShotProvider<ShotProvider.Kind.Moving> {
    private static final double LookaheadSeconds = 0.025;

    private final DeltaTime deltaTime = new DeltaTime();
    private final LinearFilter turretOmegaFilter =
            LinearFilter.movingAverage((int)(0.1 / Constants.LOOP_PERIOD_SECONDS));
    private double lastTurretAngleRads = 0;

    @Override
    public ShotParameters getParameters(
            final Pose2d robotPose,
            final Transform2d robotToTurret,
            final ChassisSpeeds robotSpeeds,
            final ShootCommands.Target target,
            final Pose2d targetPose
    ) {
        final Pose2d lookaheadRobotPose = robotPose.exp(new Twist2d(
                robotSpeeds.vxMetersPerSecond * LookaheadSeconds,
                robotSpeeds.vyMetersPerSecond *     LookaheadSeconds,
                robotSpeeds.omegaRadiansPerSecond * LookaheadSeconds
        ));
        final Pose2d lookaheadTurretPose = lookaheadRobotPose.transformBy(robotToTurret);
        final double distance = lookaheadTurretPose.getTranslation()
                .getDistance(targetPose.getTranslation());

        final Rotation2d robotAngle = lookaheadRobotPose.getRotation();
        final ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotAngle);
        final ChassisSpeeds turretFieldVelocity =
                MovingUtils.getTurretFieldSpeeds(lookaheadRobotPose, robotToTurret, fieldSpeeds);

        double timeOfFlight;
        Pose2d futureTurretPose = lookaheadTurretPose;
        double futureDistance = distance;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = ShotParameters.getTimeOfFlight(target, futureDistance);

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
                ShotParameters.getShot(target, futureDistance),
                turretAngle,
                Units.radiansToRotations(turretOmegaRadsPerSec)
        );
    }
}