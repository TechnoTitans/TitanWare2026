package frc.robot.subsystems.indexer.feeder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.ext.SubsystemExt;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemExt {
    protected static final String LogKey = "Feeder";

    public enum Goal {
        BACK_OUT(-5),
        OFF(0),
        FEED(10);

        private final double voltageSetpoint;

        Goal(final double voltageSetpoint) {
            this.voltageSetpoint = voltageSetpoint;
        }
    }

    private final FeederIO feederIO;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    private Goal desiredGoal = Goal.OFF;
    private double voltageSetpoint = 0.0;

    public Feeder(final Constants.RobotMode mode, final HardwareConstants.FeederConstants constants) {
        this.feederIO = switch (mode) {
            case REAL -> new FeederIOReal(constants);
            case SIM -> new FeederIOSim(constants);
            case REPLAY, DISABLED -> new FeederIO() {};
        };
    }

    @Override
    public void periodic() {
        final double feederPeriodicFPGATime = Timer.getFPGATimestamp();

        feederIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/VoltageSetpoint", voltageSetpoint);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - feederPeriodicFPGATime)
        );
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.OFF)
        ).withName("ToGoal: " + goal);
    }

    public boolean isTOFDetected() {
        return inputs.tofDetected;
    }

    public void setTOFDetected(final boolean isDetected) {
        feederIO.setTOFDetected(isDetected);
    }

    private void setDesiredGoal(final Goal goal) {
        desiredGoal = goal;
        setDesiredVoltage(goal.voltageSetpoint);
    }

    private void setDesiredVoltage(final double volts) {
        voltageSetpoint = volts;
        feederIO.toWheelVoltage(volts);
    }
}
