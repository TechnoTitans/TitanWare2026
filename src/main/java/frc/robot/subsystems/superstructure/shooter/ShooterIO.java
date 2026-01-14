package frc.robot.subsystems.superstructure.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public double flywheelPositionRots = 0.0;
        public double flywheelVelocityRotsPerSec = 0.0;
        public double flywheelVoltage = 0.0;
        public double flywheelTorqueCurrentAmps = 0.0;
        public double flywheelTemp = 0.0;
    }

    default void updateInputs(final ShooterIOInputs inputs) {}

    default void config() {}

    default void toFlywheelVelocity(final double velocityRotsPerSec) {}

    default void toFlywheelVoltage(final double volts) {}

    default void toFlywheelTorqueCurrent(final double torqueCurrentAmps) {}
}
