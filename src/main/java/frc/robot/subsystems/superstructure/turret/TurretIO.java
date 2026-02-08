package frc.robot.subsystems.superstructure.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    @AutoLog
    class TurretIOInputs {
        public double turretPositionRots = 0.0;
        public double turretVelocityRotsPerSec = 0.0;
        public double turretVoltage = 0.0;
        public double turretTorqueCurrentAmps = 0.0;
        public double turretTempCelsius = 0.0;

        public double leftPositionRots = 0.0;
        public double rightPositionRots = 0.0;
    }

    default void updateInputs(final TurretIOInputs inputs) {}

    default void config() {}

    default void toTurretPosition(final double positionRots) {}

    default void toTurretContinuousPosition(final double positionRots) {}

    default void toTurretVoltage(final double volts) {}

    default void toTurretTorqueCurrent(final double torqueCurrent) {}

    default void setTurretPosition(final double turretAbsolutePosition) {}

    default boolean canIntake() { return true; }

    default void intakeFuel() {}
}
