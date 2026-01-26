package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
    @AutoLog
    class SpindexerIOInputs {
        public double wheelPositionRots = 0.0;
        public double wheelVelocityRotsPerSec = 0.0;
        public double wheelVoltage = 0.0;
        public double wheelTorqueCurrentAmps = 0.0;
        public double wheelTemperatureCelsius = 0.0;
    }
    default void updateInputs(final SpindexerIOInputs inputs) {}

    default void config() {}

    default void toWheelVelocity(final double wheelVelocityRotsPerSec) {}
}
