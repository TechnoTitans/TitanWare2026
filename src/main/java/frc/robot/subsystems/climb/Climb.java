package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    protected static final String LogKey = "Climb";
    private static final double PositionToleranceRots = 0.03;
    private static final double VelocityToleranceRotsPerSec = 0.03;

    private final HardwareConstants.ClimbConstants constants;

    private final ClimbIO climbIO;
    private final ClimbIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOW;
    private Goal currentGoal = desiredGoal;

    public final Trigger atSetpoint = new Trigger(this::atSetpoint);

    public enum Goal {
        STOW(0),
        CLIMB_DOWN(0),
        EXTEND(3.7);

        private final double positionSetpointRots;

        Goal(final double positionSetpointRots) {
            this.positionSetpointRots = positionSetpointRots;
        }
    }

    public Climb(
            final Constants.RobotMode mode,
            final HardwareConstants.ClimbConstants constants
    ) {
        this.constants = constants;

        this.climbIO = switch (mode) {
            case REAL -> new ClimbIOReal(constants);
            case SIM -> new ClimbIOSim(constants);
            case REPLAY, DISABLED -> new ClimbIO() {};
        };

        this.inputs = new ClimbIOInputsAutoLogged();

        climbIO.config();
        zero();
    }

    @Override
    public void periodic() {
        final double climbPeriodicUpdateStart = Timer.getFPGATimestamp();

        climbIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            if (desiredGoal == Goal.CLIMB_DOWN) {
                climbIO.toPositionUnprofiled(desiredGoal.positionSetpointRots);
            }
            climbIO.toPosition(desiredGoal.positionSetpointRots);
            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/PositionSetpointRots", desiredGoal.positionSetpointRots);
        Logger.recordOutput(LogKey + "/ExtensionMeters", getExtensionMeters());
        Logger.recordOutput(LogKey + "/Triggers/AtLowerLimit", atLowerLimit());
        Logger.recordOutput(LogKey + "/Triggers/AtUpperLimit", atUpperLimit());
        Logger.recordOutput(LogKey + "/Triggers/AtSetpoint", atSetpoint);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - climbPeriodicUpdateStart)
        );
    }

    public boolean atGoal(final Goal goal) {
        return MathUtil.isNear(goal.positionSetpointRots, inputs.motorPositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.motorVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }

    private boolean atSetpoint() {
        return MathUtil.isNear(desiredGoal.positionSetpointRots, inputs.motorPositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.motorVelocityRotsPerSec, VelocityToleranceRotsPerSec)
                && currentGoal == desiredGoal;
    }

    public double getExtensionMeters() {
        return inputs.motorPositionRots * constants.spoolDiameterMeters();
    }

    public void zero() {
        this.climbIO.setPosition(0);
    }

    public boolean isExtended() {
        return atGoal(Goal.EXTEND);
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.STOW)
        ).withName("ToGoal: " + goal.toString());
    }

    public Command setGoal(final Goal goal) {
        return runOnce(
                () -> setDesiredGoal(goal)
        ).withName("SetGoal: " + goal.toString());
    }

    private void setDesiredGoal(final Goal goal) {
        desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }

    private boolean atLowerLimit() {
        return inputs.motorPositionRots <= constants.lowerLimitRots();
    }

    private boolean atUpperLimit() {
        return inputs.motorPositionRots >= constants.upperLimitRots();
    }
}
