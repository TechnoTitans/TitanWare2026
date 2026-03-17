package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.SubsystemExt;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemExt {
    protected static final String LogKey = "Spindexer";

    public enum Goal {
        BACK_OUT(-6),
        STOP(0),
        FEED(6);

        private final double volts;

        Goal(final double volts) {
            this.volts = volts;
        }
    }

    private final SpindexerIO spindexerIO;
    private final SpindexerIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOP;
    private double voltageSetpoint = 0.0;

    public Spindexer(final Constants.RobotMode mode, final HardwareConstants.SpindexerConstants constants) {
        this.spindexerIO = switch (mode) {
            case REAL -> new SpindexerIOReal(constants);
            case SIM -> new SpindexerIOSim(constants);
            case DISABLED, REPLAY -> new SpindexerIO() {};
        };

        this.inputs = new SpindexerIOInputsAutoLogged();
        spindexerIO.config();
    }

    @Override
    public void periodic() {
        final double spindexerPeriodicFPGATime = Timer.getFPGATimestamp();

        spindexerIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/VoltageSetpoint", desiredGoal.volts);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - spindexerPeriodicFPGATime)
        );
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.STOP)
        ).withName("ToGoal: " + goal.toString());
    }

    public Command setGoal(final Goal goal) {
        return Commands.runOnce(() -> setDesiredGoal(goal))
                .withName("SetGoal:" + goal.toString());
    }

    private void setDesiredGoal(final Goal goal) {
        desiredGoal = goal;
        setDesiredVoltage(goal.volts);
    }

    private void setDesiredVoltage(final double volts) {
        voltageSetpoint = volts;
        spindexerIO.toWheelVoltage(volts);
    }
}
