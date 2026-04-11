package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {
    class TurretIOInputs {
        public double turretPositionRots = 0.0;
        public double turretVelocityRotsPerSec = 0.0;
        public double turretVoltage = 0.0;
        public double turretTorqueCurrentAmps = 0.0;
        public double turretTempCelsius = 0.0;

        public double primaryEncoderPositionRots = 0.0;
        public double primaryEncoderAbsolutePositionRots = 0.0;
        public double secondaryEncoderPositionRots = 0.0;
        public double secondaryEncoderAbsolutePositionRots = 0.0;
    }

    default void config() {}

    default void updateInputs(final TurretIOInputs inputs) {}

    default void toTurretPosition(final double positionRots) {}

    default void toTurretContinuousPosition(final double positionRots, final double velocityRotsPerSec) {}

    default void toTurretVoltage(final double volts) {}

    default void seedTurretPosition(final Rotation2d turretPositionRots) {}
}
