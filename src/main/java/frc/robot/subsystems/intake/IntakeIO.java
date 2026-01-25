package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double rollerPositionRots = 0.0;
        public double rollerVelocityRotsPerSec = 0.0;
        public double rollerVoltage = 0.0;
        public double rollerTorqueCurrentAmps = 0.0;
        public double rollerTempCelsius = 0.0;

        public double sliderPositionRots = 0.0;
        public double sliderVelocityRotsPerSec = 0.0;
        public double sliderVoltage = 0.0;
        public double sliderTorqueCurrentAmps = 0.0;
        public double sliderTempCelsius = 0.0;

        public double encoderPositionRots = 0.0;
        public double encoderVelocityRotsPerSec = 0.0;
    }
    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see IntakeIOInputs
     * @see AutoLog
     */

    default void updateInputs(final IntakeIOInputs inputs) {}

    default void config() {}

    default void toRollerVelocity(final double velocityRotsPerSec) {}

    default void toRollerVoltage(final double volts) {}

    default void toRollerTorqueCurrent(final double torqueCurrentAmps) {}

    default void toSliderPosition(final double positionRots) {}

    default void toSliderVoltage(final double volts) {}

    default void toSliderTorqueCurrent(final double torqueCurrent) {}
}
