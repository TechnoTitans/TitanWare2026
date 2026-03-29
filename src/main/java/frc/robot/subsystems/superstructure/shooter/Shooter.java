package frc.robot.subsystems.superstructure.shooter;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.ext.SubsystemExt;
import frc.robot.utils.commands.trigger.LoggedTrigger;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemExt {
    protected static final String LogKey = "Shooter";
    private static final double VelocityToleranceRotsPerSec = 2;

    public enum Goal {
        IDLE(20),
        NO_VISION(30);

        private final double velocityRotsPerSec;

        Goal(final double velocityRotsPerSec) {
            this.velocityRotsPerSec = velocityRotsPerSec;
        }
    }

    private enum InternalGoal {
        NONE,
        DEFAULT,
        IDLE(Goal.IDLE),
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

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs;

    private final SysIdRoutine flywheelTorqueCurrentSysIdRoutine;

    private InternalGoal desiredGoal = InternalGoal.TRACKING;

    private double velocitySetpointRotsPerSec;

    public final LoggedTrigger atSetpoint;

    public Shooter(final Constants.RobotMode mode, final HardwareConstants.ShooterConstants constants) {
        this.shooterIO = switch (mode) {
            case REAL -> new ShooterIORealBangBang(constants);
            case SIM -> new ShooterIOSimBangBang(constants);
            case REPLAY, DISABLED -> new ShooterIO() {};
        };

        this.inputs = new ShooterIOInputsAutoLogged();

        final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);
        this.atSetpoint = group.t("AtSetpoint", () -> MathUtil.isNear(
                velocitySetpointRotsPerSec,
                inputs.masterVelocityRotsPerSec,
                VelocityToleranceRotsPerSec
        ));

        this.flywheelTorqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(4).per(Second),
                Amp.of(40),
                Seconds.of(10),
                shooterIO::toFlywheelTorqueCurrent
        );

        this.shooterIO.config();
        this.shooterIO.toFlywheelVelocity(velocitySetpointRotsPerSec);
    }

    @Override
    public void periodic() {
        final double shooterPeriodicUpdateStart = Timer.getFPGATimestamp();

        shooterIO.updateInputs(inputs);
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
        Logger.recordOutput(LogKey + "/VelocitySetpointRotsPerSec", velocitySetpointRotsPerSec);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - shooterPeriodicUpdateStart)
        );
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.IDLE)
        ).withName("ToGoal: " + goal);
    }

    public Command setGoal(final Goal goal) {
        return runOnce(() -> setDesiredGoal(goal))
                .withName("SetGoal: " + goal);
    }

    public Command runGoal(final Goal goal) {
        return startIdle(
                () -> setDesiredGoal(goal)
        ).withName("RunGoal");
    }

    public Command runVelocity(final DoubleSupplier velocityRotsPerSecSupplier) {
        return instantRun(
                () -> desiredGoal = InternalGoal.TRACKING,
                () -> setDesiredVelocity(velocityRotsPerSecSupplier.getAsDouble())
        ).withName("RunVelocity");
    }

    public Command flywheelTorqueCurrentSysIdCommand() {
        return makeFlywheelSysIdCommand(flywheelTorqueCurrentSysIdRoutine);
    }

    public double getVelocityRotsPerSec() {
        return inputs.masterVelocityRotsPerSec;
    }

    public boolean atSetpoint() {
        return atSetpoint.getAsBoolean();
    }

    private void setDesiredGoal(final Goal goal) {
        desiredGoal = InternalGoal.fromGoal(goal);
        setDesiredVelocity(goal.velocityRotsPerSec);
    }

    private void setDesiredVelocity(final double velocityRotsPerSec) {
        velocitySetpointRotsPerSec = velocityRotsPerSec;
        shooterIO.toFlywheelVelocity(velocityRotsPerSec);
    }

    private SysIdRoutine makeVoltageSysIdRoutine(
            final Velocity<VoltageUnit> voltageRampRate,
            final Voltage stepVoltage,
            final Time timeout,
            final Consumer<Double> voltageConsumer
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        voltageRampRate,
                        stepVoltage,
                        timeout,
                        state -> SignalLogger.writeString(String.format("%s-state", LogKey), state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> voltageConsumer.accept(voltageMeasure.in(Volts)),
                        null,
                        this
                )
        );
    }

    private SysIdRoutine makeTorqueCurrentSysIdRoutine(
            final Velocity<CurrentUnit> currentRampRate,
            final Current stepCurrent,
            final Time timeout,
            final Consumer<Double> torqueCurrentConsumer
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(currentRampRate.baseUnitMagnitude()),
                        Volts.of(stepCurrent.baseUnitMagnitude()),
                        timeout,
                        state -> SignalLogger.writeString(String.format("%s-state", LogKey), state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> torqueCurrentConsumer.accept(voltageMeasure.in(Volts)),
                        null,
                        this
                )
        );
    }

    private Command makeFlywheelSysIdCommand(final SysIdRoutine sysIdRoutine) {
        return Commands.sequence(
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(5),
                Commands.waitUntil(() -> Math.abs(inputs.masterVelocityRotsPerSec) < 0.5),
                Commands.waitSeconds(1),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(5),
                Commands.waitUntil(() -> Math.abs(inputs.masterVelocityRotsPerSec) < 0.5),
                Commands.waitSeconds(1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(5),
                Commands.waitUntil(() -> Math.abs(inputs.masterVelocityRotsPerSec) < 0.5),
                Commands.waitSeconds(1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(5)
        );
    }
}
