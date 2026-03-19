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
    private static final double PositionToleranceRots = 0.01;
    private static final double VelocityToleranceRotsPerSec = 0.05;

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

    private final HoodIO hoodIO;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private InternalGoal desiredGoal = InternalGoal.STOW;
    private InternalGoal currentGoal = InternalGoal.NONE;

    private double positionSetpointRots = 0.0;

    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);
    public final LoggedTrigger atSetpoint = group.t("AtSetpoint", this::atSetpoint);
    public final LoggedTrigger safeForTrench = group.t("SafeForTrench", () -> atGoal(Goal.STOW));
    private final LoggedTrigger atUpperLimit = group.t("AtUpperLimit", this::atUpperLimit);
    private final LoggedTrigger atLowerLimit = group.t("AtLowerLimit", this::atLowerLimit);

    public Hood(final Constants.RobotMode mode, final HardwareConstants.HoodConstants constants) {
        this.constants = constants;
        this.hoodIO = switch (mode) {
            case REAL -> new HoodIOReal(constants);
            case SIM -> new HoodIOSim(constants);
            case DISABLED, REPLAY -> new HoodIO() {
            };
        };

        this.hoodIO.config();
        this.hoodIO.zeroMotor();
    }

    @Override
    public void periodic() {
        final double hoodPeriodicUpdateStart = Timer.getFPGATimestamp();

        hoodIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (MathUtil.isNear(
                positionSetpointRots,
                inputs.hoodPositionRots,
                PositionToleranceRots
        ) && MathUtil.isNear(
                0,
                inputs.hoodVelocityRotsPerSec,
                VelocityToleranceRotsPerSec
        )) {
            currentGoal = desiredGoal;
        } else {
            currentGoal = InternalGoal.NONE;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/PositionSetpointRots", positionSetpointRots);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - hoodPeriodicUpdateStart)
        );
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.STOW)
        ).withName("ToGoal: " + goal.toString());
    }

    public Command setGoal(final Goal goal) {
        return runOnce(() -> setDesiredGoal(goal))
                .withName("SetGoal: " + goal.toString());
    }

    public Command runGoal(final Goal goal) {
        return startEnd(
                () -> setDesiredGoal(goal),
                () -> {}
        ).withName("RunGoal");
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

    private void setDesiredGoal(final Goal goal) {
        desiredGoal = InternalGoal.fromGoal(goal);
        setDesiredPosition(goal.positionRots);
    }

    private void setDesiredPosition(final double positionRots) {
        positionSetpointRots = positionRots;
        hoodIO.toHoodPosition(positionRots);
    }

    private boolean atSetpoint() {
        return currentGoal == desiredGoal;
    }

    private boolean atUpperLimit() {
        return inputs.hoodPositionRots >= constants.upperLimitRots();
    }

    private boolean atLowerLimit() {
        return inputs.hoodPositionRots <= constants.lowerLimitRots();
    }

    private boolean atGoal(final Goal goal) {
        return currentGoal == InternalGoal.fromGoal(goal);
    }
}
