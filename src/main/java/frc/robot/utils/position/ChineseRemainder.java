package frc.robot.utils.position;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

public class ChineseRemainder {
    public static double getAbsolutePosition(
            final double encoder1Gearing,
            final double encoder1Reading,
            final double encoder2Gearing,
            final double encoder2Reading,
            final int countableRotations
    ) {
        final double[] encoder1Solutions = getPossibleSolutions(encoder1Reading, encoder1Gearing, countableRotations);
        final double[] encoder2Solutions = getPossibleSolutions(encoder2Reading, encoder2Gearing, countableRotations);

        for (int i = 0; i < encoder1Solutions.length; i++) {
            if (Math.abs(encoder1Solutions[i] - encoder2Solutions[i]) < 0.01) {
                DriverStation.reportError("Found CRT Solution", false);
                return encoder1Solutions[i];
            }
        }

        DriverStation.reportError("Did not find CRT Solution", false);
        return 0;
    }

    private static double[] getPossibleSolutions(
            final double encoderReading,
            final double encoderGearing,
            final int countableRotations
    ) {
        final double[] possibleRotations = new double[countableRotations + 1];

        for (int i = 0; i <= countableRotations; i++) {
            possibleRotations[i] =
                    (i + (encoderReading / 360)) * encoderGearing;
        }

        return possibleRotations;
    }
}