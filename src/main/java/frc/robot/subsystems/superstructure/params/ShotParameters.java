package frc.robot.subsystems.superstructure.params;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.constants.Constants;

public record ShotParameters(
        Shooter shooter,
        Rotation2d turretAngle,
        double turretVelocityRotsPerSec
) {
    private static final InterpolatingTreeMap<Double, Shooter> ShotMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Shooter::interpolate);
    static {
        switch (Constants.CURRENT_MODE) {
            case REAL, REPLAY, DISABLED -> {
                ShotMap.put(1.30, new Shooter(28, 0));
                ShotMap.put(2.07, new Shooter(32, 0.005));
                ShotMap.put(2.42, new Shooter(32, 0.008));
                ShotMap.put(2.67, new Shooter(33, 0.01));
                ShotMap.put(3.07, new Shooter(35, 0.012));
                ShotMap.put(3.60, new Shooter(36, 0.014));
                ShotMap.put(4.54, new Shooter(40, 0.021));
                ShotMap.put(5.18, new Shooter(43, 0.025));
                ShotMap.put(5.84, new Shooter(44, 0.027));
                ShotMap.put(6.16, new Shooter(44, 0.041));
            }
            case SIM -> {
                ShotMap.put(1.3, new Shooter(21, 0));
                ShotMap.put(1.5, new Shooter(21, 0.008));
                ShotMap.put(2.0, new Shooter(21, 0.0175));
                ShotMap.put(2.5, new Shooter(21, 0.031));
                ShotMap.put(3.0, new Shooter(22.5, 0.034));
                ShotMap.put(3.5, new Shooter(23, 0.045));
                ShotMap.put(4.0, new Shooter(24, 0.0475));
                ShotMap.put(4.5, new Shooter(25, 0.05));
                ShotMap.put(5.0, new Shooter(26, 0.0525));
                ShotMap.put(5.5, new Shooter(27, 0.055));
                ShotMap.put(6.0, new Shooter(27.75, 0.0565));
            }
        }
    }

    private static final InterpolatingDoubleTreeMap TimeOfFlightMap = new InterpolatingDoubleTreeMap();
    static {
        switch (Constants.CURRENT_MODE) {
            case REAL, REPLAY, DISABLED -> {
                // TODO: redo
                TimeOfFlightMap.put(1.989, 1.2);
                TimeOfFlightMap.put(2.209, 1.3);
                TimeOfFlightMap.put(2.823, 1.4);
                TimeOfFlightMap.put(3.17, 1.5);
                TimeOfFlightMap.put(3.664d, 1.6);
                TimeOfFlightMap.put(4.643, 1.7);
            }
            case SIM -> {
                TimeOfFlightMap.put(1.31, 1.06);
                TimeOfFlightMap.put(2.17, 1.0);
                TimeOfFlightMap.put(2.72, 1.02);
                TimeOfFlightMap.put(3.51, 1.06);
                TimeOfFlightMap.put(4.63, 1.2);
                TimeOfFlightMap.put(4.9, 1.24);
                TimeOfFlightMap.put(5.73, 1.31);
            }
        }
    }

    public static Shooter getShot(final double distanceMeters) {
        return ShotMap.get(distanceMeters);
    }

    public static double getTimeOfFlight(final double distanceMeters) {
        return TimeOfFlightMap.get(distanceMeters);
    }

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
