package frc.robot.subsystems.indexer.hopper;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.ext.SubsystemExt;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemExt {
    protected static final String LogKey = "Spindexer";

    public enum Goal {
        STOP(0),
        FEED(6);

        private final double volts;

        Goal(final double volts) {
            this.volts = volts;
        }
    }

    private final HopperIO hopperIO;
    private final HopperIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOP;
    private double voltageSetpoint = 0.0;

    public Hopper(final Constants.RobotMode mode, final HardwareConstants.HopperConstants constants) {
        this.hopperIO = switch (mode) {
            case REAL -> new HopperIOReal(constants);
            case SIM -> new HopperIOSim(constants);
            case DISABLED, REPLAY -> new HopperIO() {};
        };

        this.inputs = new HopperIOInputsAutoLogged();

        this.hopperIO.config();
        this.hopperIO.toRollersVoltage(voltageSetpoint);
    }

    @Override
    public void periodic() {
        final double hopperPeriodicFPGATime = Timer.getFPGATimestamp();

        hopperIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/VoltageSetpoint", voltageSetpoint);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - hopperPeriodicFPGATime)
        );
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.STOP)
        ).withName("ToGoal: " + goal);
    }

    public Command setGoal(final Goal goal) {
        return Commands.runOnce(() -> setDesiredGoal(goal))
                .withName("SetGoal:" + goal);
    }
    private void setDesiredGoal(final Goal goal) {
        desiredGoal = goal;
        setDesiredVoltage(goal.volts);
    }

    private void setDesiredVoltage(final double volts) {
        voltageSetpoint = volts;
        hopperIO.toRollersVoltage(volts);
    }
}
