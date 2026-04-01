package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class CRT {
    public static long[] bezout(final long a, final long b) {
        long old_r = a, r = b;
        long old_s = 1, s = 0;
        long old_t = 0, t = 1;

        while (r != 0) {
            long quotient = old_r / r;

            long temp_r = r;
            r = old_r - quotient * r;
            old_r = temp_r;

            long temp_s = s;
            s = old_s - quotient * s;
            old_s = temp_s;

            long temp_t = t;
            t = old_t - quotient * t;
            old_t = temp_t;
        }

        return new long[]{old_s, old_t, old_r};
    }

    public static Rotation2d solve(
            final int outputGearTeeth,
            final double absolutePosition0,
            final int gearTeeth0,
            final double absolutePosition1,
            final int gearTeeth1,
            final double upperBound
    ) {
        final long[] bez = bezout(gearTeeth0, gearTeeth1);
        final long u = bez[0];
        final long v = bez[1];
        final long gcd = bez[2];

        double a = absolutePosition0 * gearTeeth0;
        double b = absolutePosition1 * gearTeeth1;

        final double offset = MathUtil.inputModulus(a - b, -0.5, 0.5);
        a -= 0.5 * offset;
        b += 0.5 * offset;

        final double q = ((double)gearTeeth0 * gearTeeth1) / gcd;
        final double x = ((a * v * gearTeeth1) + (b * u * gearTeeth0)) / gcd;

        final double pos = (x % q) / outputGearTeeth;
        final double period = ((double) gearTeeth0 * gearTeeth1 / gcd) / outputGearTeeth;
        final double wrappedPos = MathUtil.inputModulus(
                pos,
                upperBound - period,
                upperBound
        );

        return Rotation2d.fromRotations(wrappedPos);
    }
}