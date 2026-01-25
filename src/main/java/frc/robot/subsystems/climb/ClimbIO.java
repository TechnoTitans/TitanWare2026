package frc.robot.subsystems.climb;

import frc.robot.subsystems.superstructure.hood.HoodIO;
import org.littletonrobotics.junction.AutoLog;

public class ClimbIO  {
    @AutoLog
    class ClimbIOInputs {
        public double motorPositionRots = 0.0;
        public double motorVelocity = 0.0;
        public double motorVoltage = 0.0;
        public double motorTorqueCurrentAmps = 0.0;
        public double motorTemperatureCelsius = 0.0;

        public double encoderPositionsRots = 0.0;
    }

    default void updateInputs(final ClimbIOInputs inputs) {}

    default void config() {}

    default void toMotorPosition(final double positionRots) {}

    default void toMotorVoltage(final double volts) {}

    default void toMotorTorqueCurrent(final double torqueCurrentAmps) {}

}
