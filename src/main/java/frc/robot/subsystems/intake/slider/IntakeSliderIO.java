package frc.robot.subsystems.intake.slider;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeSliderIO {
    @AutoLog
    class IntakeArmIOInputs {
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
     * @see IntakeArmIOInputs
     * @see AutoLog
     */

    default void updateInputs(final IntakeArmIOInputs inputs) {}

    default void config() {}

    default void toPosition(final double positionRots) {}
}