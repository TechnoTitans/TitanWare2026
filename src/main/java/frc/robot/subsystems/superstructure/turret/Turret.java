package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.ext.SubsystemExt;
import frc.robot.utils.commands.trigger.LoggedTrigger;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class Turret extends SubsystemExt {
    protected static final String LogKey = "Turret";
    private static final double PositionToleranceRots = 0.05;
    private static final double VelocityToleranceRotsPerSec = 0.25;

    public enum Goal {
        STOW(0),
        NO_VISION(0.5);

        private final double positionSetpointRots;

        Goal(final double positionSetpointRots) {
            this.positionSetpointRots = positionSetpointRots;
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

    @SuppressWarnings("FieldCanBeLocal")
    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);
    private final HardwareConstants.TurretConstants constants;

    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

    private InternalGoal desiredGoal = InternalGoal.STOW;
    private double positionSetpointRots = 0.0;
    private double velocitySetpointRotsPerSec = 0.0;

    private boolean positionSeeded = false;

    public final LoggedTrigger atSetpoint;
    private final SysIdRoutine voltageSysIdRoutine;

    public Turret(final Constants.RobotMode mode, final HardwareConstants.TurretConstants constants) {
        this.constants = constants;
        this.turretIO = switch (mode) {
            case REAL -> new TurretIOReal(constants);
            case SIM -> new TurretIOSim(constants);
            case REPLAY, DISABLED -> new TurretIO() {};
        };

        this.atSetpoint = group.t("AtSetpoint", () -> MathUtil.isNear(
                positionSetpointRots,
                inputs.turretPositionRots,
                PositionToleranceRots
        ) && MathUtil.isNear(
                velocitySetpointRotsPerSec,
                inputs.turretVelocityRotsPerSec,
                VelocityToleranceRotsPerSec
        ));

        this.voltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(0.5).per(Second),
                Volts.of(2),
                Seconds.of(6)
        );
    }

    @Override
    public void periodic() {
        final double turretPeriodicUpdateStart = Timer.getFPGATimestamp();

        turretIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (!positionSeeded && MathUtil.isNear(0, inputs.turretVelocityRotsPerSec, 1e-3)) {
            final Rotation2d position = CRT.solve(
                    constants.drivenTurretGearTeeth(),
                    inputs.primaryEncoderAbsolutePositionRots,
                    constants.primaryCANcoderGearTeeth(),
                    inputs.secondaryEncoderAbsolutePositionRots,
                    constants.secondaryCANcoderGearTeeth(),
                    constants.forwardLimitRots()
            );
            turretIO.seedTurretPosition(position);
            positionSeeded = true;
        }

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
        Logger.recordOutput(LogKey + "/VelocitySetpointRotsPerSec", velocitySetpointRotsPerSec);

        Logger.recordOutput(LogKey + "/AtUpperLimit", atUpperLimit());
        Logger.recordOutput(LogKey + "/AtLowerLimit", atLowerLimit());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - turretPeriodicUpdateStart)
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

    public Command runPositionWithVelocity(final DoubleSupplier positionRots, final DoubleSupplier velocityRotsPerSec) {
        return instantRun(
                () -> desiredGoal = InternalGoal.TRACKING,
                () -> setDesiredPositionWithVelocity(positionRots.getAsDouble(), velocityRotsPerSec.getAsDouble())
        ).withName("RunPositionWithVelocity");
    }

    public Command voltageSysIdCommand() {
        return makeSysIdCommand(voltageSysIdRoutine);
    }

    public Rotation2d getTurretPosition() {
        return Rotation2d.fromRotations(inputs.turretPositionRots);
    }

    public Transform2d getOffsetFromCenter() {
        return constants.offsetFromCenter();
    }

    public boolean atSetpoint() {
        return atSetpoint.getAsBoolean();
    }

    private void setDesiredGoal(final Goal goal) {
        desiredGoal = InternalGoal.fromGoal(goal);
        setDesiredPosition(desiredGoal.goal.positionSetpointRots);
    }

    private void setDesiredPosition(final double positionRots) {
        positionSetpointRots = positionRots;
        velocitySetpointRotsPerSec = 0;
        turretIO.toTurretPosition(positionSetpointRots);
    }

    private void setDesiredPositionWithVelocity(final double positionRots, final double velocityRotsPerSec) {
        positionSetpointRots = optimizeWrap(positionRots);
        velocitySetpointRotsPerSec = velocityRotsPerSec;
        turretIO.toTurretContinuousPosition(positionSetpointRots, velocityRotsPerSec);
    }

    private boolean atUpperLimit() {
        return inputs.turretPositionRots >= constants.forwardLimitRots();
    }

    private boolean atLowerLimit() {
        return inputs.turretPositionRots <= constants.reverseLimitRots();
    }

    private double optimizeWrap(final double targetPositionRots) {
        final double forwardLimitRots = constants.forwardLimitRots();
        final double reverseLimitRots = constants.reverseLimitRots();
        final double currentPositionRots = inputs.turretPositionRots;

        final double wrappedPositionRots = currentPositionRots + MathUtil.inputModulus(
                targetPositionRots - currentPositionRots,
                -0.5,
                0.5
        );

        if (wrappedPositionRots >= reverseLimitRots && wrappedPositionRots <= forwardLimitRots) {
            return wrappedPositionRots;
        }

        final double fullRotationForward = wrappedPositionRots + 1.0;
        final double fullRotationBackward = wrappedPositionRots - 1.0;

        final boolean isForwardWithinBound = fullRotationForward >= reverseLimitRots
                && fullRotationForward <= forwardLimitRots;
        final boolean isBackwardWithinBound = fullRotationBackward >= reverseLimitRots
                && fullRotationBackward <= forwardLimitRots;

        if (isForwardWithinBound && isBackwardWithinBound) {
            final double forwardTravelRots = Math.abs(fullRotationForward - currentPositionRots);
            final double backwardTravelRots = Math.abs(fullRotationBackward - currentPositionRots);

            return forwardTravelRots <= backwardTravelRots ? fullRotationForward : fullRotationBackward;
        } else if (isForwardWithinBound) {
            return fullRotationForward;
        } else if (isBackwardWithinBound) {
            return fullRotationBackward;
        }

        return MathUtil.clamp(wrappedPositionRots, reverseLimitRots, forwardLimitRots);
    }

    private SysIdRoutine makeVoltageSysIdRoutine(
            final Velocity<VoltageUnit> voltageRampRate,
            final Voltage stepVoltage,
            final Time timeout
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        voltageRampRate,
                        stepVoltage,
                        timeout,
                        state -> SignalLogger.writeString(String.format("%s-state", LogKey), state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> turretIO.toTurretVoltage(
                                voltageMeasure.in(Volts)
                        ),
                        null,
                        this
                )
        );
    }

    private Command makeSysIdCommand(final SysIdRoutine sysIdRoutine) {
        return Commands.sequence(
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(this::atUpperLimit),
                Commands.waitSeconds(1),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(this::atLowerLimit),
                Commands.waitSeconds(1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(this::atUpperLimit),
                Commands.waitSeconds(1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(this::atLowerLimit)
        );
    }
}