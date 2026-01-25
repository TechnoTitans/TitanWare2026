package frc.robot.subsystems.superstructure.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    class HopperIOInputs {
        public double rollerPositionRots = 0.0;
        public double rollerVelocityRotsPerSec = 0.0;
        public double rollerVoltage = 0.0;
        public double rollerTorqueCurrentAmps = 0.0;
        public double rollerTemperatureCelsius = 0.0;
    }
    default void updateInputs(final HopperIOInputs inputs) {}

    default void config() {}

    default void toRollerVelocity(final double rollerVelocityRotsPerSec) {}
}
