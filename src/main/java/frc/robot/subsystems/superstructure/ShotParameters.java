package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;

public record ShotParameters(
        Shooter shooter,
        Rotation2d turretAngle,
        double turretVelocityRotsPerSec
) {
    public record Shooter(
            double shooterVelocityRotsPerSec,
            double hoodPositionRots
    ) implements Interpolatable<Shooter> {
        @Override
        public Shooter interpolate(final Shooter endValue, final double t) {
            return new Shooter(
                    MathUtil.interpolate(
                            shooterVelocityRotsPerSec,
                            endValue.shooterVelocityRotsPerSec,
                            t
                    ),
                    MathUtil.interpolate(
                            hoodPositionRots,
                            endValue.hoodPositionRots,
                            t
                    )
            );
        }
    }
}
