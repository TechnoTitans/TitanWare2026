package frc.robot.subsystems.superstructure.hood;

public interface HoodIO {
    class HoodIOInputs {
        public double hoodPositionRots = 0.0;
        public double hoodVelocityRotsPerSec = 0.0;
        public double hoodVoltage = 0.0;
        public double hoodTorqueCurrentAmps = 0.0;
        public double hoodTempCelsius = 0.0;
    }

    default void config() {}

    default void updateInputs(final HoodIOInputs inputs) {}

    default void toHoodPosition(final double positionRots) {}

    default void zero() {}
}
