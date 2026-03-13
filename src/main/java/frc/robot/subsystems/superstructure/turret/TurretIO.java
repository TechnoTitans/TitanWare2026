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

        public double primaryEncoderPositionRots = 0.0;
        public double secondaryEncoderPositionRots = 0.0;
    }

    default void updateInputs(final TurretIOInputs inputs) {}

    default void config() {}

    default void toTurretContinuousPosition(final double positionRots, final double velocityRotsPerSec) {}

    default void setPosition(final double turretPositionRots) {}
}
