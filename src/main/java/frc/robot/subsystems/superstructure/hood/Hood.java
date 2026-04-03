package frc.robot.subsystems.superstructure.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.ext.SubsystemExt;
import frc.robot.utils.commands.trigger.LoggedTrigger;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

public class Hood extends SubsystemExt {
    protected static final String LogKey = "Hood";
    private static final double PositionToleranceRots = 0.0125;
    private static final double VelocityToleranceRotsPerSec = 0.15;

    private final HardwareConstants.HoodConstants constants;

    public enum Goal {
        STOW(0),
        NO_VISION(0.02);

        private final double positionRots;

        Goal(final double positionRots) {
            this.positionRots = positionRots;
        }
    }

    private enum InternalGoal {
        NONE,
        DEFAULT,
        STOW(Goal.STOW),
        NO_VISION(Goal.NO_VISION),
        TRACKING;

        public static final HashMap<Goal, InternalGoal> GoalToInternal = new HashMap<>();
        static {
            for (final InternalGoal goal : InternalGoal.values()) {
                if (goal.goal != null) {
                    GoalToInternal.put(goal.goal, goal);
                }
            }
        }

        public static InternalGoal fromGoal(final Goal goal) {
            final InternalGoal internalGoal = GoalToInternal.get(goal);

            return internalGoal == null ? DEFAULT : internalGoal;
        }

        public final Goal goal;

        InternalGoal(final Goal goal) {
            this.goal = goal;
        }

        InternalGoal() {
            this(null);
        }
    }

    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final HoodIO hoodIO;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private InternalGoal desiredGoal = InternalGoal.STOW;
    private double positionSetpointRots = 0.0;

    public final LoggedTrigger atSetpoint;
    public final LoggedTrigger safeForTrench = group.t("SafeForTrench", () -> atGoal(Goal.STOW));

    public Hood(final Constants.RobotMode mode, final HardwareConstants.HoodConstants constants) {
        this.constants = constants;
        this.hoodIO = switch (mode) {
            case REAL -> new HoodIOReal(constants);
            case SIM -> new HoodIOSim(constants);
            case REPLAY, DISABLED -> new HoodIO() {};
        };
        this.hoodIO.zero();

        this.atSetpoint = group.t("AtSetpoint", () -> MathUtil.isNear(
                positionSetpointRots,
                inputs.hoodPositionRots,
                PositionToleranceRots
        ) && MathUtil.isNear(
                0,
                inputs.hoodVelocityRotsPerSec,
                VelocityToleranceRotsPerSec
        ));
    }

    @Override
    public void periodic() {
        final double hoodPeriodicUpdateStart = Timer.getFPGATimestamp();

        hoodIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        final InternalGoal currentGoal;
        if (atSetpoint()) {
            currentGoal = desiredGoal;
        } else {
            currentGoal = InternalGoal.NONE;
        }

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal);
        Logger.recordOutput(LogKey + "/AtSetpoint", atSetpoint);
        Logger.recordOutput(LogKey + "/PositionSetpointRots", positionSetpointRots);

        Logger.recordOutput(LogKey + "/AtUpperLimit", atUpperLimit());
        Logger.recordOutput(LogKey + "/AtLowerLimit", atLowerLimit());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - hoodPeriodicUpdateStart)
        );
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.STOW)
        ).withName("ToGoal: " + goal);
    }

    public Command setGoal(final Goal goal) {
        return runOnce(() -> setDesiredGoal(goal))
                .withName("SetGoal: " + goal);
    }

    public Command runGoal(final Goal goal) {
        return startIdle(() -> setDesiredGoal(goal))
                .withName("RunGoal");
    }

    public Command runPosition(final DoubleSupplier positionRotsSupplier) {
        return instantRun(
                () -> desiredGoal = InternalGoal.TRACKING,
                () -> setDesiredPosition(positionRotsSupplier.getAsDouble())
        ).withName("RunPosition");
    }

    public Rotation2d getHoodPosition() {
        return Rotation2d.fromRotations(inputs.hoodPositionRots);
    }

    public boolean atSetpoint() {
        return atSetpoint.getAsBoolean();
    }

    private void setDesiredGoal(final Goal goal) {
        desiredGoal = InternalGoal.fromGoal(goal);
        setDesiredPosition(goal.positionRots);
    }

    private void setDesiredPosition(final double positionRots) {
        positionSetpointRots = positionRots;
        hoodIO.toHoodPosition(positionRots);
    }

    private boolean atUpperLimit() {
        return inputs.hoodPositionRots >= constants.upperLimitRots();
    }

    private boolean atLowerLimit() {
        return inputs.hoodPositionRots <= constants.lowerLimitRots();
    }

    private boolean atGoal(final Goal goal) {
        return desiredGoal == InternalGoal.fromGoal(goal)
                && atSetpoint();
    }
}
