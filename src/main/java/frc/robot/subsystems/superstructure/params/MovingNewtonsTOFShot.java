package frc.robot.subsystems.superstructure.params;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.ShootCommands;
import frc.robot.constants.Constants;
import frc.robot.utils.control.DeltaTime;

public class MovingNewtonsTOFShot implements ShotProvider<ShotProvider.Kind.Moving> {
    private static final double Epsilon = 0.001;
    private static final double VelocityTolerance = 0.005;
    private static final double LookaheadSeconds = 0.01;

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
                robotSpeeds.vyMetersPerSecond * LookaheadSeconds,
                robotSpeeds.omegaRadiansPerSecond * LookaheadSeconds
        ));
        final Pose2d lookaheadTurretPose = lookaheadRobotPose.transformBy(robotToTurret);

        final Translation2d toGoal = targetPose.getTranslation()
                .minus(lookaheadTurretPose.getTranslation());
        final double distance = toGoal.getNorm();
        final Translation2d targetDirection = toGoal.div(distance);

        final double initialTimeOfFlight = ShotParameters.getTimeOfFlight(target, distance);
        final double initialVelocity = distance / initialTimeOfFlight;

        final Rotation2d robotAngle = lookaheadRobotPose.getRotation();
        final ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotAngle);
        final ChassisSpeeds turretFieldSpeeds =
                MovingUtils.getTurretFieldSpeeds(lookaheadRobotPose, robotToTurret, fieldSpeeds);

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
                    / ShotParameters.getTimeOfFlight(target, futureDistance - Epsilon);
            final double highVel = (futureDistance + Epsilon)
                    / ShotParameters.getTimeOfFlight(target, futureDistance + Epsilon);
            final double velPrime = (highVel - lowVel) / (2 * Epsilon);

            futureDistance -= (futureVelocity - requiredVelocity) / velPrime;
            timeOfFlight = ShotParameters.getTimeOfFlight(target, futureDistance);
            futureVelocity = futureDistance / timeOfFlight;

            if (Math.abs(futureVelocity - requiredVelocity) <= VelocityTolerance) {
                break;
            }
        }

        final double dt = deltaTime.get();
        final Rotation2d turretAngle = turretFieldAngle
                .minus(robotAngle);
        final double turretAngleRads = turretAngle.getRadians();
        final double turretOmegaRadsPerSec = turretOmegaFilter.calculate(
                MathUtil.angleModulus(turretAngleRads - lastTurretAngleRads)
                        / dt
        );
        lastTurretAngleRads = turretAngleRads;

        return new ShotParameters(
                ShotParameters.getShot(target, futureDistance),
                turretFieldAngle
                        .minus(robotAngle),
                Units.radiansToRotations(turretOmegaRadsPerSec)
        );
    }
}