package frc.robot.subsystems.superstructure.calculation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

public record ShotCalculation(
    double turretRotationRots,
    double turretSpeedRotsPerSec,
    ShooterCalculation shooterCalculation
) {
    public record ShooterCalculation(
            double shooterVelocityRotsPerSec,
            double hoodPositionRots
    ) implements Interpolatable<ShooterCalculation> {
        @Override
        public ShooterCalculation interpolate(final ShooterCalculation endValue, final double t) {
            return new ShooterCalculation(
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
