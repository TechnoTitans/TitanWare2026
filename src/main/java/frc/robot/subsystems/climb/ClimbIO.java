package frc.robot.subsystems.climb;

import frc.robot.subsystems.superstructure.hood.HoodIO;
import org.littletonrobotics.junction.AutoLog;

public class ClimbIO  {
    @AutoLog
    class ClimbIOInputs {
        public double climbPositionRots = 0.0;
        public double climbVelocity = 0.0;
        public double climbVoltage = 0.0;
        public double climbTorqueCurrentAmps = 0.0;
        public double climbTempteratureCelsius = 0.0;

        public double encoderPositionsRots = 0.0;
    }

    default void updateInputs(final HoodIO.HoodIOInputs inputs) {}

    default void config() {}

    default void toClimbPosition(final double positionRots) {}

    default void toClimbContinuousPosition(final double positionRots) {}

    default void toClimbVoltage(final double volts) {}

    default void toClimbTorqueCurrent(final double torqueCurrentAmps) {}

}
