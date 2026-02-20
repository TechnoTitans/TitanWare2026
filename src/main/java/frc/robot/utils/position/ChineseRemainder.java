package frc.robot.utils.position;

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
                return encoder1Solutions[i];
            }
        }

        throw new RuntimeException("No matching solution found between encoders");
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