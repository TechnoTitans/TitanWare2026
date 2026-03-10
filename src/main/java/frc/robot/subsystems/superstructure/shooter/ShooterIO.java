package frc.robot.subsystems.superstructure.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public double masterPositionRots = 0;
        public double masterVelocityRotsPerSec = 0;
        public double masterVoltage = 0;
        public double masterTorqueCurrentAmps = 0;
        public double masterTempCelsius = 0;

        public double followerPositionRots = 0;
        public double followerVelocityRotsPerSec = 0;
        public double followerVoltage = 0;
        public double followerTorqueCurrentAmps = 0;
        public double followerTempCelsius = 0;
    }

    default void updateInputs(final ShooterIOInputs inputs) {}

    default void config() {}

    default void toShooterVelocity(final double shooterVelocityRotsPerSec) {}

    default void toShooterVoltage(final double shooterVolts) {}
}
