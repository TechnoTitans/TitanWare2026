package frc.robot.utils.position;

import java.util.HashSet;
import java.util.Set;

public class ChineseRemainder {
    final double encoder1Position;
    final double encoder2Position;

    private final double encoder1Gearing;
    private final double encoder2Gearing;

    private final int countableRotations;

    public ChineseRemainder(
            final double encoder1Position,
            final double encoder2Position,
            final double encoder1Gearing,
            final double encoder2Gearing,
            final int countableRotations) {
        this.encoder1Position = encoder1Position;
        this.encoder2Position = encoder2Position;
        this.encoder1Gearing = encoder1Gearing;
        this.encoder2Gearing = encoder2Gearing;
        this.countableRotations = countableRotations;
    }

    public double getAbsolutePosition() {
        final Set<Double> encoder1PossibleSolutions =
                possibleSolutions(encoder1Gearing, encoder1Position);

        final Set<Double> encoder2PossibleSolutions =
                possibleSolutions(encoder2Gearing, encoder2Position);

        encoder1PossibleSolutions.retainAll(encoder2PossibleSolutions);

        return encoder1PossibleSolutions.iterator().next();
    }

    private Set<Double> possibleSolutions(
            final double encoderGearing,
            final double encoderReading) {
        final Set<Double> possibleSolutions = new HashSet<>();

        for (int i = 0; i < countableRotations; i++) {
            possibleSolutions.add(
                    (i + 1) + (encoderReading) * encoderGearing
            );
        }

        return possibleSolutions;
    }
}
