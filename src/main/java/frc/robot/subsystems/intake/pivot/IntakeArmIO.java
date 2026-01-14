package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {
    @AutoLog
    class IntakeArmIOInputs {
        public double pivotPositionRots = 0.0;
        public double pivotVelocityRotsPerSec = 0.0;
        public double pivotVoltage = 0.0;
        public double pivotTorqueCurrentAmps = 0.0;
        public double pivotTempCelsius = 0.0;
        public double coralTOFDistanceMeters = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see IntakeArmIOInputs
     * @see AutoLog
     */

    default void updateInputs(final IntakeArmIOInputs inputs) {}

    default void config() {}

    default void toRollerVelocity(final double velocityRotsPerSec) {}

    default void toRollerVoltage(final double volts) {}

    default void toRollerTorqueCurrent(final double torqueCurrentAmps) {}

    default void setTOFDistance(final double distanceMeters) {}
}