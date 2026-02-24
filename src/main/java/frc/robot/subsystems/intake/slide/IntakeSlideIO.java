package frc.robot.subsystems.intake.slide;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeSlideIO {
    @AutoLog
    class IntakeSlideIOInputs {
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

        public double averagePositionRots = 0.0;
        public double differentialPositionRots = 0.0;
    }

    default void updateInputs(final IntakeSlideIOInputs inputs) {}

    default void toSlidePosition(final double positionRots) {}

    default void toSlidePositionUnprofiled(final double positionRots, final double velocityRotsPerSec) {}

    default void toSlideVoltage(final double volts) {}
    default void holdSlidePosition(final double positionRots) {}

    default void home() {}

    default void zeroMotors() {}
}
