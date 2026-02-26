package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog
    class ClimbIOInputs {
        public double motorPositionRots = 0.0;
        public double motorVelocityRotsPerSec = 0.0;
        public double motorVoltage = 0.0;
        public double motorTorqueCurrentAmps = 0.0;
        public double motorTempCelsius = 0.0;
    }

    default void updateInputs(final ClimbIOInputs inputs) {}

    default void config() {}

    default void toPosition(final double positionRots) {}

    default void toPositionClimb(final double positionRots) {}

    default void toVoltage(final double volts) {}

    default void toTorqueCurrent(final double torqueCurrentAmps) {}

    default void setPosition(final double positionRots) {}
}
