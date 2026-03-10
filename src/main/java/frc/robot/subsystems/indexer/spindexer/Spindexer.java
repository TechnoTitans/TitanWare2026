package frc.robot.subsystems.indexer.spindexer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.SubsystemExt;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemExt {
    protected static final String LogKey = "Spindexer";

    public enum ControlMode {
        Voltage,
        TorqueCurrent
    }

    public enum Goal {
        OFF(ControlMode.Voltage, 0),
        AGITATE(ControlMode.Voltage, 4),
        FEED(ControlMode.Voltage, 11);

        public final ControlMode controlMode;
        public final double output;

        Goal(final ControlMode controlMode, final double output) {
            this.controlMode = controlMode;
            this.output = output;
        }
    }

    private final SpindexerIO spindexerIO;
    private final SpindexerIOInputsAutoLogged inputs;

    private ControlMode controlMode;
    private double setpointOutput;
    private Goal desiredGoal = Goal.OFF;

    public Spindexer(final Constants.RobotMode mode, final HardwareConstants.SpindexerConstants constants) {
        this.spindexerIO = switch (mode) {
            case REAL -> new SpindexerIOReal(constants);
            case SIM -> new SpindexerIOSim(constants);
            case REPLAY, DISABLED -> new SpindexerIO() {};
        };

        this.inputs = new SpindexerIOInputsAutoLogged();
        this.spindexerIO.config();
    }

    @Override
    public void periodic() {
        final double spindexerPeriodicUpdateStart = Timer.getFPGATimestamp();

        spindexerIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/ControlMode", controlMode);
        Logger.recordOutput(LogKey + "/SetpointOutput", setpointOutput);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - spindexerPeriodicUpdateStart)
        );
    }

    private void setVoltageImpl(final double volts) {
        controlMode = ControlMode.Voltage;
        setpointOutput = volts;
        spindexerIO.toVoltage(volts);
    }

    private void setTorqueCurrentImpl(final double torqueCurrentAmps) {
        controlMode = ControlMode.TorqueCurrent;
        setpointOutput = torqueCurrentAmps;
        spindexerIO.toTorqueCurrent(torqueCurrentAmps);
    }

    private void setGoalImpl(final Goal goal) {
        desiredGoal = goal;
        switch (goal.controlMode) {
            case Voltage -> setVoltageImpl(goal.output);
            case TorqueCurrent -> setTorqueCurrentImpl(goal.output);
        }
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setGoalImpl(goal),
                () -> setGoalImpl(Goal.OFF)
        );
    }
}
