package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.SubsystemExt;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Turret extends SubsystemExt {
    protected static final String LogKey = "Turret";
    private static final double PositionToleranceRots = 0.025; //TODO: Check
    private static final double VelocityToleranceRotsPerSec = 0.25;

    public enum Goal {
        IDLE(0);

        public final double positionRots;

        Goal(final double positionRots) {
            this.positionRots = positionRots;
        }
    }

    private enum InternalGoal {
        NONE,
        IDLE(Goal.IDLE),
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

    private final HardwareConstants.TurretConstants constants;

    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged inputs;

    private InternalGoal desiredGoal = InternalGoal.IDLE;
    private InternalGoal currentGoal = InternalGoal.NONE;

    private double positionSetpointRots;
    private double velocitySetpointRotsPerSec;

    public Turret(final Constants.RobotMode mode, final HardwareConstants.TurretConstants constants) {
        this.constants = constants;
        this.turretIO = switch (mode) {
            case REAL -> new TurretIOReal(constants);
            case SIM -> new TurretIOSim(constants);
            case REPLAY, DISABLED -> new TurretIO() {};
        };

        this.inputs = new TurretIOInputsAutoLogged();

        this.turretIO.config();
        this.turretIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        final Rotation2d absolutePosition = CRT.findAbsolutePosition(
                constants.drivenTurretGearTeeth(),
                inputs.primaryCANcoderPositionRots,
                constants.primaryCANcoderGearTeeth(),
                inputs.secondaryCANcoderPositionRots,
                constants.secondaryCANcoderGearTeeth()
        );
        this.turretIO.seedTurretPosition(absolutePosition);
    }

    @Override
    public void periodic() {
        final double turretPeriodicUpdateStart = Timer.getFPGATimestamp();

        turretIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (MathUtil.isNear(
                positionSetpointRots,
                inputs.motorPositionRots,
                PositionToleranceRots
        ) && MathUtil.isNear(
                velocitySetpointRotsPerSec,
                inputs.motorVelocityRotsPerSec,
                VelocityToleranceRotsPerSec
        )) {
            currentGoal = desiredGoal;
        } else {
            currentGoal = InternalGoal.NONE;
        }

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal);
        Logger.recordOutput(LogKey + "/PositionSetpointRots", positionSetpointRots);
        Logger.recordOutput(LogKey + "/VelocitySetpointRotsPerSec", velocitySetpointRotsPerSec);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - turretPeriodicUpdateStart)
        );
    }

    public Transform2d getOffsetFromCenter() {
        return constants.offsetFromCenter();
    }

    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(inputs.motorPositionRots);
    }

    public boolean atSetpoint() {
        return desiredGoal == currentGoal;
    }

    private void setPositionImpl(final double positionRots, final double velocityRotsPerSec) {
        positionSetpointRots = positionRots;
        velocitySetpointRotsPerSec = velocityRotsPerSec;
        turretIO.trackTurretPosition(positionSetpointRots, velocityRotsPerSec);
    }

    private void setGoalImpl(final Goal goal) {
        desiredGoal = InternalGoal.fromGoal(goal);
        positionSetpointRots = goal.positionRots;
        velocitySetpointRotsPerSec = 0;
        turretIO.toTurretPosition(positionSetpointRots);
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setGoalImpl(goal),
                () -> setGoalImpl(Goal.IDLE)
        );
    }

    public Command runGoal(final Goal goal) {
        return startEnd(() -> setGoalImpl(goal), () -> {});
    }

    public Command toPosition(
            final Supplier<Rotation2d> positionSupplier,
            final DoubleSupplier velocitySupplier
    ) {
        return instantRunEnd(
                () -> desiredGoal = InternalGoal.TRACKING,
                () -> setPositionImpl(positionSupplier.get().getRotations(), velocitySupplier.getAsDouble()),
                () -> setGoalImpl(Goal.IDLE)
        );
    }

    public Command runPosition(
            final Supplier<Rotation2d> positionSupplier,
            final DoubleSupplier velocitySupplier
    ) {
        return instantRun(
                () -> desiredGoal = InternalGoal.TRACKING,
                () -> setPositionImpl(positionSupplier.get().getRotations(), velocitySupplier.getAsDouble())
        );
    }
}
