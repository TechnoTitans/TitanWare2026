package frc.robot.subsystems.superstructure.shooter;

public interface ShooterIO {
    class ShooterIOInputs {
        public double masterPositionRots = 0.0;
        public double masterVelocityRotsPerSec = 0.0;
        public double masterVoltage = 0.0;
        public double masterTorqueCurrentAmps = 0.0;
        public double masterTempCelsius = 0.0;

        public double followerPositionRots = 0.0;
        public double followerVelocityRotsPerSec = 0.0;
        public double followerVoltage = 0.0;
        public double followerTorqueCurrentAmps = 0.0;
        public double followerTempCelsius = 0.0;
    }

    default void config() {}

    default void updateInputs(final ShooterIOInputs inputs) {}

    default void toFlywheelVelocity(final double velocityRotsPerSec) {}

    default void toFlywheelVoltage(final double volts) {}

    default void toFlywheelTorqueCurrent(final double torqueCurrentAmps) {}
}
