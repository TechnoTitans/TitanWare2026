package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
    @AutoLog
    class IntakeRollersIOInputs {
        public double rollerPositionRots = 0;
        public double rollerVelocityRotsPerSec = 0;
        public double rollerVoltage = 0;
        public double rollerTorqueCurrentAmps = 0;
        public double rollerTempCelsius = 0;
    }

    default void updateInputs(final IntakeRollersIOInputs inputs) {}

    default void config() {}

    default void toIntakeVelocity(final double intakeVelocityRotsPerSec) {}

    default void toIntakeVoltage(final double intakeVolts) {}
}
