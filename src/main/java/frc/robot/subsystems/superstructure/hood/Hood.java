package frc.robot.subsystems.superstructure.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.LoggedTrigger;
import frc.robot.utils.commands.SubsystemExt;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Objects;
import java.util.function.DoubleSupplier;

public class Hood extends SubsystemExt {
    protected static final String LogKey = "Hood";
    private static final double PositionToleranceRots = 0.005; //TODO: Check
    private static final double VelocityToleranceRotsPerSec = 0.1;

    public enum Goal {
        STOW(0);

        public final double positionRots;

        Goal(final double positionRots) {
            this.positionRots = positionRots;
        }
    }

    private enum InternalGoal {
        NONE,
        STOW(Goal.STOW),
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
            return Objects.requireNonNull(GoalToInternal.get(goal));
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
    private final HoodIOInputsAutoLogged inputs;

    private InternalGoal desiredGoal = InternalGoal.STOW;
    private InternalGoal currentGoal = InternalGoal.NONE;

    private double positionSetpointRots;

    public final LoggedTrigger safeForTrench = atGoal(Hood.Goal.STOW);

    public Hood(final Constants.RobotMode mode, final HardwareConstants.HoodConstants constants) {
        this.hoodIO = switch (mode) {
            case REAL -> new HoodIOReal(constants);
            case SIM -> new HoodIOSim(constants);
            case REPLAY, DISABLED -> new HoodIO() {};
        };

        this.inputs = new HoodIOInputsAutoLogged();

        this.hoodIO.config();
        this.hoodIO.setPosition(0);
    }

    @Override
    public void periodic() {
        final double shooterPeriodicUpdateStart = Timer.getFPGATimestamp();

        hoodIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (MathUtil.isNear(positionSetpointRots, inputs.pivotPositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.pivotVelocityRotsPerSec, VelocityToleranceRotsPerSec)
        ) {
            currentGoal = desiredGoal;
        } else {
            currentGoal = InternalGoal.NONE;
        }

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal);
        Logger.recordOutput(LogKey + "/PositionSetpointRots", positionSetpointRots);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - shooterPeriodicUpdateStart)
        );
    }

    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(inputs.pivotPositionRots);
    }

    public boolean atSetpoint() {
        return desiredGoal == currentGoal;
    }

    private boolean atGoal(final InternalGoal goal) {
        return currentGoal == goal;
    }

    public LoggedTrigger atGoal(final Goal goal) {
        return group.t("AtGoal", () -> atGoal(InternalGoal.fromGoal(goal)));
    }

    private void setPositionImpl(final double positionRots) {
        positionSetpointRots = positionRots;
        hoodIO.toHoodPosition(positionSetpointRots);
    }

    private void setGoalImpl(final Goal goal) {
        desiredGoal = InternalGoal.fromGoal(goal);
        setPositionImpl(goal.positionRots);
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setGoalImpl(goal),
                () -> setGoalImpl(Goal.STOW)
        );
    }

    public Command runGoal(final Goal goal) {
        return startEnd(() -> setGoalImpl(goal), () -> {});
    }

    public Command toPosition(final DoubleSupplier positionRotsSupplier) {
        return instantRunEnd(
                () -> desiredGoal = InternalGoal.TRACKING,
                () -> setPositionImpl(positionRotsSupplier.getAsDouble()),
                () -> setGoalImpl(Goal.STOW)
        );
    }

    public Command runPosition(final DoubleSupplier positionRotsSupplier) {
        return instantRun(
                () -> desiredGoal = InternalGoal.TRACKING,
                () -> setPositionImpl(positionRotsSupplier.getAsDouble())
        );
    }
}
