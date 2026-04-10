package frc.robot.subsystems.superstructure.params;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.ShootCommands;
import frc.robot.constants.Constants;

public record ShotParameters(
        Shooter shooter,
        Rotation2d turretAngle,
        double turretVelocityRotsPerSec
) {
    private static final InterpolatingTreeMap<Double, Shooter> HubShotMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Shooter::interpolate);
    private static final InterpolatingTreeMap<Double, Shooter> FerryShotMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Shooter::interpolate);
    static {
        switch (Constants.CURRENT_MODE) {
            case REAL, REPLAY, DISABLED -> {
                HubShotMap.put(1.30, new Shooter(27.5, 0));
                HubShotMap.put(2.07, new Shooter(30, 0.005));
                HubShotMap.put(2.42, new Shooter(31, 0.008));
                HubShotMap.put(2.67, new Shooter(32, 0.01));
                HubShotMap.put(3.07, new Shooter(33.5, 0.012));
                HubShotMap.put(3.60, new Shooter(34, 0.014));
//                HubShotMap.put(4.232, new Shooter(38, 0.22));
                HubShotMap.put(4.54, new Shooter(38, 0.023));
                HubShotMap.put(5.18, new Shooter(38.5, 0.035));
                HubShotMap.put(5.84, new Shooter(40, 0.038));
                HubShotMap.put(6.16, new Shooter(40, 0.041));

                FerryShotMap.put(2.0, new Shooter(34, 0.1));
                FerryShotMap.put(3.0, new Shooter(34, 0.1));
                FerryShotMap.put(4.0, new Shooter(35, 0.08));
                FerryShotMap.put(5.0, new Shooter(36, 0.07));
                FerryShotMap.put(6.0, new Shooter(39, 0.06));
            }
            case SIM -> {
                HubShotMap.put(1.3, new Shooter(21, 0));
                HubShotMap.put(1.5, new Shooter(21, 0.008));
                HubShotMap.put(2.0, new Shooter(21, 0.0175));
                HubShotMap.put(2.5, new Shooter(21, 0.031));
                HubShotMap.put(3.0, new Shooter(22.5, 0.034));
                HubShotMap.put(3.5, new Shooter(23, 0.045));
                HubShotMap.put(4.0, new Shooter(24, 0.0475));
                HubShotMap.put(4.5, new Shooter(25, 0.05));
                HubShotMap.put(5.0, new Shooter(26, 0.0525));
                HubShotMap.put(5.5, new Shooter(27, 0.055));
                HubShotMap.put(6.0, new Shooter(27.75, 0.0565));

                FerryShotMap.put(2.0, new Shooter(22, 0.1));
                FerryShotMap.put(3.0, new Shooter(22, 0.1));
                FerryShotMap.put(4.0, new Shooter(23, 0.08));
                FerryShotMap.put(5.0, new Shooter(24, 0.07));
                FerryShotMap.put(6.0, new Shooter(26, 0.06));
            }
        }
    }

    private static final InterpolatingDoubleTreeMap TimeOfFlightMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap FerryTimeOfFlightMap = new InterpolatingDoubleTreeMap();
    static {
        switch (Constants.CURRENT_MODE) {
            case REAL, REPLAY, DISABLED -> {
                TimeOfFlightMap.put(1.647, 1.08);
                TimeOfFlightMap.put(2.246, 1.05);
                TimeOfFlightMap.put(3.252, 1.35);
                TimeOfFlightMap.put(3.991, 1.34);
                TimeOfFlightMap.put(4.8, 1.4);
                TimeOfFlightMap.put(5.322, 1.47);

                FerryTimeOfFlightMap.put(2.014, 1.16);
                FerryTimeOfFlightMap.put(3.339, 1.06);
                FerryTimeOfFlightMap.put(3.722464, 1.2);
                FerryTimeOfFlightMap.put(4.541, 1.27);
                FerryTimeOfFlightMap.put(4.780, 1.36);
                FerryTimeOfFlightMap.put(5.995, 1.4);
            }
            case SIM -> {
                TimeOfFlightMap.put(1.31, 1.06);
                TimeOfFlightMap.put(2.17, 1.0);
                TimeOfFlightMap.put(2.72, 1.02);
                TimeOfFlightMap.put(3.51, 1.06);
                TimeOfFlightMap.put(4.63, 1.2);
                TimeOfFlightMap.put(4.9, 1.24);
                TimeOfFlightMap.put(5.73, 1.31);

                FerryTimeOfFlightMap.put(2.0, 1.1);
                FerryTimeOfFlightMap.put(3.0, 1.2);
                FerryTimeOfFlightMap.put(4.0, 1.3);
                FerryTimeOfFlightMap.put(5.0, 1.4);
            }
        }
    }

    public static Shooter getShot(final ShootCommands.Target target, final double distanceMeters) {
        return switch (target) {
            case HUB -> HubShotMap.get(distanceMeters);
            case FERRY, FERRY_BLOCKED -> FerryShotMap.get(distanceMeters);
        };
    }

    public static double getTimeOfFlight(final ShootCommands.Target target, final double distanceMeters) {
        return switch (target) {
            case HUB -> TimeOfFlightMap.get(distanceMeters);
            case FERRY, FERRY_BLOCKED -> FerryTimeOfFlightMap.get(distanceMeters);
        };
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
