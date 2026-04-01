package frc.robot.subsystems.indexer.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    class FeederIOInputs {
        public double wheelPositionRots = 0.0;
        public double wheelVelocityRotsPerSec = 0.0;
        public double wheelVoltage = 0.0;
        public double wheelTorqueCurrentAmps = 0.0;
        public double wheelTempCelsius = 0.0;

        public boolean tofDetected = false;
    }

    default void config() {}

    default void updateInputs(final FeederIOInputs inputs) {}

    default void toWheelTorqueCurrent(final double torqueCurrentAmps) {}

    default void toWheelVoltage(final double volts) {}

    default void setTOFDetected(final boolean isDetected) {}
}
