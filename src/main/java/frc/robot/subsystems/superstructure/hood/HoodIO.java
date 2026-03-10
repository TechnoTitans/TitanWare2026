package frc.robot.subsystems.superstructure.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    class HoodIOInputs {
        public double pivotPositionRots = 0;
        public double pivotVelocityRotsPerSec = 0;
        public double pivotVoltage = 0;
        public double pivotTorqueCurrentAmps = 0;
        public double pivotTempCelsius = 0;
    }

    default void updateInputs(final HoodIOInputs inputs) {}

    default void config() {}

    default void toHoodPosition(final double hoodPositionRots) {}

    default void toHoodVoltage(final double hoodVolts) {}

    default void setPosition(final double positionRots) {}
}
