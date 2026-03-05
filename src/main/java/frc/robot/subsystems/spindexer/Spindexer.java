package frc.robot.subsystems.spindexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
    protected static final String LogKey = "Spindexer";
    private static final double VelocityToleranceRotsPerSec = 0.002;

    private final SpindexerIO spindexerIO;
    private final SpindexerIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOP;
    private Goal currentGoal = desiredGoal;

    public enum Goal {
        STOP(0),
        FEED(50);

        private final double velocitySetpointRotsPerSec;

        Goal(final double velocitySetpointRotsPerSec) {
            this.velocitySetpointRotsPerSec = velocitySetpointRotsPerSec;
        }
    }

    public Spindexer(final Constants.RobotMode mode, final HardwareConstants.SpindexerConstants constants) {
        this.spindexerIO = switch (mode) {
            case REAL -> new SpindexerIOReal(constants);
            case SIM -> new SpindexerIOSim(constants);
            case DISABLED, REPLAY -> new SpindexerIO() {};
        };

        this.inputs = new SpindexerIOInputsAutoLogged();

        spindexerIO.config();

        spindexerIO.toWheelVelocity(desiredGoal.velocitySetpointRotsPerSec);
    }

    @Override
    public void periodic() {
        final double spindexerPeriodicFPGATime = Timer.getFPGATimestamp();

        spindexerIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            spindexerIO.toWheelVelocity(desiredGoal.velocitySetpointRotsPerSec);
            currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/VelocitySetpointRotsPerSec", desiredGoal.velocitySetpointRotsPerSec);
        Logger.recordOutput(LogKey + "/AtSetpoint", atVelocitySetpoint());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - spindexerPeriodicFPGATime)
        );
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.STOP)
        ).withName("ToGoal: " + goal.toString());
    }

    public Command setGoal(final Goal goal) {
        return Commands.runOnce(
                () -> setDesiredGoal(goal)
        ).withName("SetGoal:" + goal.toString());
    }

    private void setDesiredGoal(final Goal goal) {
        desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", goal.toString());
    }

    private boolean atVelocitySetpoint() {
        return currentGoal == desiredGoal
                && MathUtil.isNear(desiredGoal.velocitySetpointRotsPerSec, inputs.wheelVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }
}
