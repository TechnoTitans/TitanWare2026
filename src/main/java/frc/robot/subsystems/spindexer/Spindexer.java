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
        INTAKE(4),
        FEED(5),
        EJECT(-5);

        private final double rollerVelocitySetpoint;

        Goal(final double rollerVelocitySetpoint) {
            this.rollerVelocitySetpoint = rollerVelocitySetpoint;
        }

        public double getRollerVelocitySetpoint() {
            return rollerVelocitySetpoint;
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

        spindexerIO.toWheelVelocity(desiredGoal.getRollerVelocitySetpoint());
    }

    @Override
    public void periodic() {
        final double HopperPeriodicFPGATime = Timer.getFPGATimestamp();

        spindexerIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            spindexerIO.toWheelVelocity(desiredGoal.getRollerVelocitySetpoint());
            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/HopperVelocityRotsPerSec", desiredGoal.getRollerVelocitySetpoint());
        Logger.recordOutput(LogKey + "/Triggers/AtVelocitySetpoint", atVelocitySetpoint());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - HopperPeriodicFPGATime)
        );
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                () -> setGoal(goal),
                () -> setGoal(Goal.STOP)
        ).withName("ToGoal: " + goal);
    }

    public Command setGoal(final Goal goal) {
        return Commands.runOnce(
                () -> setDesiredGoal(goal)
        ).withName("SetGoal:" + goal);
    }

    private void setDesiredGoal(final Goal desiredGoal) {
        this.desiredGoal = desiredGoal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }

    private boolean atVelocitySetpoint() {
        return MathUtil.isNear(desiredGoal.rollerVelocitySetpoint, inputs.wheelVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }
}
