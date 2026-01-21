package frc.robot.subsystems.superstructure.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    class HoodIOInputs {
        public double hoodPositionRots = 0.0;
        public double hoodVelocityRotsPerSec = 0.0;
        public double hoodVoltage = 0.0;
        public double hoodTorqueCurrentAmps = 0.0;
        public double hoodTempCelsius = 0.0;
    }


    default void updateInputs(final HoodIOInputs inputs) {}

    default void config() {}

    default void toHoodPosition(final double positionRots) {}

    default void toHoodContinuousPosition(final double positionRots) {}

    default void toHoodVoltage(final double volts) {}

    default void toHoodTorqueCurrent(final double torqueCurrentAmps) {}

    default void home() {}

    default void zeroMotor() {}
}
