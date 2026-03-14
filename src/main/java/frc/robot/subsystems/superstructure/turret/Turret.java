package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class Turret extends SubsystemBase {
    protected static final String LogKey = "Turret";

    private static final double PositionToleranceRots = 0.02;
    private static final double VelocityToleranceRotsPerSec = 0.02;

    private final HardwareConstants.TurretConstants constants;
    private final DoubleSupplier turretVelocitySupplier;

    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged inputs;

    private final SysIdRoutine voltageSysIdRoutine;
    private final SysIdRoutine torqueCurrentSysIdRoutine;

    private Goal desiredGoal = Goal.TRACKING;
    private Goal currentGoal = desiredGoal;

    public final Trigger atSetpoint = new Trigger(this::atSetpoint);

    public enum Goal {
        TRACKING(0, true);

        private final boolean isDynamic;

        private double positionSetpointRots;

        Goal(final double positionSetpointRots, final boolean isDynamic) {
            this.positionSetpointRots = positionSetpointRots;
            this.isDynamic = isDynamic;
        }

        public void changeTurretPositionRots(final double desiredTurretPositionRots) {
            if (isDynamic) {
                this.positionSetpointRots = desiredTurretPositionRots;
            }
        }
    }

    public Turret(
            final Constants.RobotMode mode,
            final HardwareConstants.TurretConstants constants,
            final DoubleSupplier turretVelocitySupplier
    ) {
        this.constants = constants;
        this.turretIO = switch (mode) {
            case REAL -> new TurretIOReal(constants);
            case SIM -> new TurretIOSim(constants);
            case REPLAY, DISABLED -> new TurretIO() {};
        };

        this.turretVelocitySupplier = turretVelocitySupplier;

        this.inputs = new TurretIOInputsAutoLogged();
        this.turretIO.config();

        this.voltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(2).per(Second),
                Volts.of(3),
                Seconds.of(6)
        );
        this.torqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(2).per(Second),
                Amps.of(8),
                Seconds.of(6)
        );


        turretIO.setPosition(0);
    }

    @Override
    public void periodic() {
        final double turretPeriodicUpdateStart = Timer.getFPGATimestamp();

        turretIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            currentGoal = desiredGoal;
            turretIO.toTurretContinuousPosition(desiredGoal.positionSetpointRots, 0);
        }

        if (desiredGoal.isDynamic) {
            turretIO.toTurretContinuousPosition(
                    desiredGoal.positionSetpointRots,
                    Units.radiansToRotations(turretVelocitySupplier.getAsDouble())
            );
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/PositionSetpointRots", desiredGoal.positionSetpointRots);

        Logger.recordOutput(LogKey + "/Triggers/AtSetpoint", atSetpoint());
        Logger.recordOutput(LogKey + "/Triggers/AtLowerLimit", atLowerLimit());
        Logger.recordOutput(LogKey + "/Triggers/AtUpperLimit", atUpperLimit());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - turretPeriodicUpdateStart)
        );
    }

    public Transform2d getOffsetFromCenter() {
        return constants.offsetFromCenter();
    }

    public Command setGoalCommand(final Goal goal) {
        return runOnce(() -> setGoal(goal));
    }

    public void setGoal(final Goal goal) {
        desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal);
        Logger.recordOutput(LogKey + "/DesiredGoal", goal);
    }

    public void updatePositionSetpoint(final double desiredTurretPosition) {
        desiredGoal.changeTurretPositionRots(optimizeWrap(desiredTurretPosition));
    }

    public Rotation2d getTurretPosition() {
        return Rotation2d.fromRotations(inputs.turretPositionRots);
    }

    private boolean atSetpoint() {
        return currentGoal == desiredGoal
                && MathUtil.isNear(desiredGoal.positionSetpointRots, inputs.turretPositionRots, PositionToleranceRots)
                && MathUtil.isNear(turretVelocitySupplier.getAsDouble(), inputs.turretVelocityRotsPerSec, VelocityToleranceRotsPerSec);
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

    private SysIdRoutine makeTorqueCurrentSysIdRoutine(
            final Velocity<CurrentUnit> currentRampRate,
            final Current stepCurrent,
            final Time timeout
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(currentRampRate.baseUnitMagnitude()),
                        Volts.of(stepCurrent.baseUnitMagnitude()),
                        timeout,
                        state -> SignalLogger.writeString(String.format("%s-state", LogKey), state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> turretIO.toTurretTorqueCurrent(
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

    public Command voltageSysIdCommand() {
        return makeSysIdCommand(voltageSysIdRoutine);
    }

    public Command torqueCurrentSysIdCommand() {
        return makeSysIdCommand(torqueCurrentSysIdRoutine);
    }
}