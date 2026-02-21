package frc.robot.subsystems.superstructure.calculator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;

public class ShotCalculationStructs {
    public record HoodShooterCalculation(
            Rotation2d hoodRotation,
            double flywheelVelocity,
            double shotTime
    ) implements Interpolatable<HoodShooterCalculation> {

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

    public static final double FerryXBoundary = Units.inchesToMeters(205);

    public static final InterpolatingTreeMap<Double, HoodShooterCalculation> shotDataMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            HoodShooterCalculation.interpolator
    );

    //TODO: Temp values

    static {
        shotDataMap.put(1d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(5),
                20,
                1
        ));

        shotDataMap.put(1.5d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(20),
                20,
                1.05
        ));

        shotDataMap.put(2d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(15),
                20,
                1.1
        ));

        shotDataMap.put(2.5d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(18),
                20,
                1.15
        ));

        shotDataMap.put(3d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(19),
                20,
                1.2
        ));

        shotDataMap.put(3.5d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(20),
                20,
                1.25
        ));

        shotDataMap.put(4d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(22),
                20,
                1.3
        ));

        shotDataMap.put(4.5d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(25),
                20,
                1.35
        ));

        shotDataMap.put(5d, new HoodShooterCalculation(
                Rotation2d.fromDegrees(26),
                20,
                1.4
        ));
    }

    public record ShotCalculation(
            Rotation2d desiredTurretRotation,
            HoodShooterCalculation hoodShooterCalculation,
            ShotCalculator.Target target
    ) {}
}
