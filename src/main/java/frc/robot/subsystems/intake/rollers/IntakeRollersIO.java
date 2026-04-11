package frc.robot.subsystems.intake.rollers;

public interface IntakeRollersIO {
    class IntakeRollerIOInputs {
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

    default void updateInputs(final IntakeRollerIOInputs inputs) {}

    default void toRollersVoltage(final double volts) {}

    default void toRollersTorqueCurrent(final double torqueCurrentAmps) {}

    default void toRollersVelocity(final double velocityRotsPerSec) {}
}
