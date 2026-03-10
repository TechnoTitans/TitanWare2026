package frc.robot.subsystems.indexer.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
        @AutoLog
        class SpindexerIOInputs {
            public double spindexerPositionRots = 0;
            public double spindexerVelocityRotsPerSec = 0;
            public double spindexerVoltage = 0;
            public double spindexerTorqueCurrentAmps = 0;
            public double spindexerTempCelsius = 0;
        }

        default void updateInputs(final SpindexerIOInputs inputs) {}

        default void config() {}

        default void toTorqueCurrent(final double torqueCurrentAmps) {}

        default void toVoltage(final double volts) {}
}
