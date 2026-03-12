package frc.robot.utils.position;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class ChineseRemainder {
    private static final long CRT_RESOLUTION = 1_000_000;

    private static long[] egcd(final long a, final long b) {
        long ma = a;
        long mb = b;

        long x = 0;
        long y = 1;
        long lastX = 1, lastY = 0;

        int i = 0;
        while (mb != 0) {
            if (i > 100) {
                throw new RuntimeException("EGCD exceeded iteration limit");
            }

            final long f = ma / mb;
            final long remainder = ma % mb;
            ma = mb;
            mb = remainder;

            final long tx = x;
            x = lastX - (f * x);
            lastX = tx;

            final long ty = y;
            y = lastY - (f * y);
            lastY = ty;

            i++;
        }

        return new long[] {ma, lastX, lastY};
    }

    private static long crt(final long a, final long m, final long b, final long n) {
        final long[] vals = egcd(m, n);
        final long g = vals[0];
        final long x = vals[1];

        if ((a - b) % g != 0) {
            DriverStation.reportError("No solutions exist for the given inputs.", true);
            return 0;
//            throw new IllegalArgumentException("No solutions exist for the given inputs.");
        }

        final long lcm = (m / g) * n;
        final long k = ((b - a) / g) % (n / g);
        final long t = (a + m * (k * x % (n / g))) % lcm;
        if (t < 0) {
            return t + lcm;
        }

        return t;
    }

    public static Rotation2d findAbsolutePosition(
            final int outputGearTeeth,
            final double absolutePosition0,
            final int gearTeeth0,
            final double absolutePosition1,
            final int gearTeeth1
    ) {
        final long scale0 = gearTeeth0 * CRT_RESOLUTION;
        final long scale1 = gearTeeth1 * CRT_RESOLUTION;

        final long round0 = Math.round(absolutePosition0 * scale0);
        final long round1 = Math.round(absolutePosition1 * scale1);

        final double outputScale = outputGearTeeth * CRT_RESOLUTION;

        return Rotation2d.fromRotations(
                crt(round0, scale0, round1, scale1) / outputScale
        );
    }
}