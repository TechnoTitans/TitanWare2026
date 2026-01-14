package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.robot.constants.FieldConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ShotCalculator {
    protected static String LogKey = "ShotCalculator";

    public record HoodShooterCalculation(
            Rotation2d hoodRotation,
            double flywheelVelocity,
            double shotTime
    )  implements Interpolatable<HoodShooterCalculation> {
        public static final Interpolator<HoodShooterCalculation> interpolator = HoodShooterCalculation::interpolate;


        @Override
        public HoodShooterCalculation interpolate(final HoodShooterCalculation endShotCalculation, final double t) {
            return new HoodShooterCalculation(
                    this.hoodRotation.interpolate(endShotCalculation.hoodRotation, t),
                    MathUtil.interpolate(this.flywheelVelocity, endShotCalculation.flywheelVelocity, t),
                    MathUtil.interpolate(this.shotTime, endShotCalculation.shotTime, t)
            );
        }
    }

    private static final InterpolatingTreeMap<Double, HoodShooterCalculation> shotDataMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            HoodShooterCalculation.interpolator
    );

    static {
        shotDataMap.put(1d, new HoodShooterCalculation(
                Rotation2d.fromRadians(0.437138669),
                10,
                0.1
        ));

        shotDataMap.put(1.5d, new HoodShooterCalculation(
                Rotation2d.fromRadians(0.611352924),
                10,
                0.15
        ));

        shotDataMap.put(2d, new HoodShooterCalculation(
                Rotation2d.fromRadians(0.75159462),
                10,
                0.2
        ));

        shotDataMap.put(2.5d, new HoodShooterCalculation(
                Rotation2d.fromRadians(0.86282925),
                10,
                0.20
        ));

        shotDataMap.put(3d, new HoodShooterCalculation(
                Rotation2d.fromRadians(0.951177756),
                10,
                0.25
        ));

        shotDataMap.put(3.5d, new HoodShooterCalculation(
                Rotation2d.fromRadians(1.022015817),
                10,
                0.3
        ));

        shotDataMap.put(4d, new HoodShooterCalculation(
                Rotation2d.fromRadians(1.079542321),
                10,
                0.35
        ));

        shotDataMap.put(4.5d, new HoodShooterCalculation(
                Rotation2d.fromRadians(1.126894805),
                10,
                0.4
        ));

        shotDataMap.put(5d, new HoodShooterCalculation(
                Rotation2d.fromRadians(1.166387399),
                10,
                0.50
        ));
    }

    public record ShotCalculation(
            Rotation2d desiredTurretRotation,
            HoodShooterCalculation hoodShooterCalculation
    ) {}

    public static ShotCalculation getShotCalculation(final Supplier<Pose2d> swervePoseSupplier) {
        return getShotCalculation(swervePoseSupplier.get());
    }

    private static ShotCalculation getShotCalculation(final Pose2d swervePose) {
        final Rotation2d differenceInAngle = FieldConstants.hubCenter.minus(swervePose.getTranslation()).getAngle();
        Logger.recordOutput(LogKey + "DifferenceInAngle", differenceInAngle);

        final Rotation2d desiredTurretRotation = differenceInAngle.minus(swervePose.getRotation());
        Logger.recordOutput(LogKey + "DesiredTurretRotation", desiredTurretRotation);

        final double distanceToTarget = FieldConstants.hubCenter.getDistance(swervePose.getTranslation());

        return new ShotCalculation(
                desiredTurretRotation,
                shotDataMap.get(
                        distanceToTarget
                )
        );
    }
}
