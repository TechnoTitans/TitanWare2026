package frc.robot.subsystems.intakeRoller;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
    @AutoLog
    class IntakeRollerIOInputs {
        public double rollerPositionRots = 0.0;
        public double rollerVelocityRotsPerSec = 0.0;
        public double rollerVoltage = 0.0;
        public double rollerTorqueCurrentAmps = 0.0;
        public double rollerTempCelsius = 0.0;
    }

    default void updateInputs(final IntakeRollerIOInputs inputs) {}

    default void config() {}

    default void toRollerVelocity(final double velocityRotsPerSec) {}

    default void toRollerVoltage(final double volts) {}

    default void toRollerTorqueCurrent(final double torqueCurrentAmps) {}
}
