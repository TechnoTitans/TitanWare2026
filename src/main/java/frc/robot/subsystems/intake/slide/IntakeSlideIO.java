package frc.robot.subsystems.intake.slide;

import com.ctre.phoenix6.mechanisms.MechanismState;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeSlideIO {
    @AutoLog
    class IntakeSlideIOInputs {
        public MechanismState slideState;

        public double slideAveragePositionRots = 0;
        public double slideAverageVelocityRotsPerSec = 0;
        public double slideDifferentialPositionRots = 0;
        public double slideDifferentialVelocityRotsPerSec = 0;

        public double masterPositionRots = 0;
        public double masterVelocityRotsPerSec = 0;
        public double masterVoltage = 0;
        public double masterTorqueCurrentAmps = 0;
        public double masterTempCelsius = 0;

        public double followerPositionRots = 0;
        public double followerVelocityRotsPerSec = 0;
        public double followerVoltage = 0;
        public double followerTorqueCurrentAmps = 0;
        public double followerTempCelsius = 0;
    }

    default void updateInputs(final IntakeSlideIOInputs inputs) {}

    default void toSlidePosition(final double slidePositionRots) {}

    default void holdSlidePosition(final double slidePositionRots) {}

    default void toSlidePositionVelocity(final double slidePositionRots, final double slideVelocityRotsPerSec) {}

    default void toSlideTorqueCurrent(final double slideTorqueCurrentAmps) {}
}
