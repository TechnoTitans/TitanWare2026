package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    class FeederIOInputs {
        public double rollerPositionRots = 0.0;
        public double rollerVelocityRotsPerSec = 0.0;
        public double rollerVoltage = 0.0;
        public double rollerTorqueCurrentAmps = 0.0;
        public double rollerTempCelsius = 0.0;
    }

    default void updateInputs(final FeederIOInputs inputs) {}

    default void config() {}

    default void toRollerVelocity(final double rollerVelocityRotsPerSec) {}
}
