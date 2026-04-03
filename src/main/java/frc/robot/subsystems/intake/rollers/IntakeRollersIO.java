package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollersIO {
    @AutoLog
    class IntakeRollerIOInputs {
        public double rollersPositionRots = 0.0;
        public double rollersVelocityRotsPerSec = 0.0;
        public double rollersVoltage = 0.0;
        public double rollersTorqueCurrentAmps = 0.0;
        public double rollersTempCelsius = 0.0;
    }

    default void config() {}

    default void updateInputs(final IntakeRollerIOInputs inputs) {}

    default void toRollersVoltage(final double volts) {}

    default void toRollersTorqueCurrent(final double torqueCurrentAmps) {}
}
