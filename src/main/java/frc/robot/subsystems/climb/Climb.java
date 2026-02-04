package frc.robot.subsystems.climb;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class Climb extends SubsystemBase {
    protected static final String LogKey = "Climb";
    private static final double PositionToleranceRots = 0.0;
    private static final double VelocityToleranceRotsPerSec = 0.0;

    private final HardwareConstants.ClimbConstants constants;

    private final ClimbIO climbIO;
    private final ClimbIOInputsAutoLogged inputs;

    private final SysIdRoutine voltageSysIdRoutine;
    private final SysIdRoutine torqueCurrentSysIdRoutine;

    private Goal desiredGoal = Goal.STOW;
    private Goal currentGoal = desiredGoal;

    private final PositionSetpoint setpoint;
    private final PositionSetpoint climbLowerLimit;
    private final PositionSetpoint climbUpperLimit;

    public final Trigger atSetpoint = new Trigger(this::atPositionSetpoint);
    public final Trigger atLowerLimit = new Trigger(this::atLowerLimit);
    public final Trigger atUpperLimit = new Trigger(this::atUpperLimit);

    public static class PositionSetpoint {
        public double climbPositionRots = 0.0;

        public PositionSetpoint withClimbPositionRots(final double climbPositionRots) {
            this.climbPositionRots = climbPositionRots;
            return this;
        }

        public static boolean atSetpoint(
                final double setpointClimbPositionRots,
                final double climbPositionRots,
                final double climbVelocityRotsPerSec
        ) {
            return MathUtil.isNear(setpointClimbPositionRots, climbPositionRots, PositionToleranceRots)
                    && MathUtil.isNear(0, climbVelocityRotsPerSec, VelocityToleranceRotsPerSec);
        }

        public boolean atSetpoint(final double climbPositionRots, final double climbVelocityRotsPerSec) {
            return PositionSetpoint.atSetpoint(
                    this.climbPositionRots,
                    climbPositionRots,
                    climbVelocityRotsPerSec
            );
        }
    }

    public enum Goal {
        DYNAMIC(0),
        STOW(0),
        CLIMB_DOWN(0),
        HP(0),
        AUTO_L1(0);

        private final double positionGoalMeters;
        Goal(final double positionGoalMeters) {
            this.positionGoalMeters = positionGoalMeters;
        }

        public double getPositionGoalRots(final HardwareConstants.ClimbConstants constants) {
            return positionGoalMeters;
        }

        public double getPositionGoalMeters() {
            return positionGoalMeters;
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

        this.voltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(0).per(Second),
                Volts.of(0),
                Seconds.of(0)
        );
        this.torqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(0).per(Second),
                Amps.of(0),
                Seconds.of(0)
        );

        this.setpoint = new PositionSetpoint()
                .withClimbPositionRots(desiredGoal.getPositionGoalRots(constants));
        this.climbLowerLimit = new PositionSetpoint().withClimbPositionRots(constants.lowerLimitRots());
        this.climbUpperLimit = new PositionSetpoint().withClimbPositionRots(constants.upperLimitRots());

        this.climbIO.config();
        this.home();
        this.climbIO.toPosition(setpoint.climbPositionRots);
    }

    @Override
    public void periodic() {
        final double climbPeriodicUpdateStart = RobotController.getFPGATime();

        climbIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            if (desiredGoal != Goal.DYNAMIC) {
                setpoint.climbPositionRots = desiredGoal.getPositionGoalRots(constants);
            }
        }

        this.currentGoal = desiredGoal;


        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/PositionSetpoint/PositionRots", setpoint.climbPositionRots);
        Logger.recordOutput(LogKey + "/AtPositionSetpoint", atPositionSetpoint());
        Logger.recordOutput(LogKey + "/AtLowerLimit", atLowerLimit());
        Logger.recordOutput(LogKey + "/AtUpperLimit", atUpperLimit());
        Logger.recordOutput(LogKey + "/ExtensionMeters", getExtensionMeters());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(RobotController.getFPGATime() - climbPeriodicUpdateStart)
        );
    }

    public boolean atGoal(final Goal goal) {
        return PositionSetpoint.atSetpoint(
                goal.getPositionGoalRots(constants),
                inputs.motorPositionRots,
                inputs.motorVelocityRotsPerSec
        );
    }

    private boolean atPositionSetpoint() {
        return setpoint.atSetpoint(inputs.motorPositionRots, inputs.motorVelocityRotsPerSec)
                && currentGoal == desiredGoal;
    }

    private boolean atLowerLimit() {
        return inputs.motorPositionRots <= climbLowerLimit.climbPositionRots;
    }

    private boolean atUpperLimit() {
        return inputs.motorPositionRots >= climbUpperLimit.climbPositionRots;
    }

    public double getExtensionMeters() {
        return inputs.motorPositionRots;
    }

    public void home() {
        this.climbIO.setPosition(0);
    }

    public Command toGoal(final Goal desiredGoal){
        return runEnd(
                () -> setGoal(desiredGoal),
                () -> setGoal(Goal.CLIMB_DOWN)
        ).withName("ToGoal");
    }

    public void setGoal(final Goal goal) {
        this.desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }

    public Command runPositionMetersCommand(final DoubleSupplier positionMeters) {
        return run(() -> {
            this.desiredGoal = Goal.DYNAMIC;
            setpoint.climbPositionRots = positionMeters.getAsDouble();
            climbIO.toPosition(setpoint.climbPositionRots);
        });
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
                        voltageMeasure -> climbIO.toVoltage(
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
                        voltageMeasure -> climbIO.toTorqueCurrent(
                                voltageMeasure.in(Volts)
                        ),
                        null,
                        this
                )

        );
    }

    private Command makeSysIdCommand(final SysIdRoutine sysIdRoutine) {
        return Commands.sequence(
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(atUpperLimit),
                Commands.waitSeconds(0.1),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(atLowerLimit),
                Commands.waitSeconds(0.1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(atUpperLimit),
                Commands.waitSeconds(0.1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(atLowerLimit)
        );
    }

    public Command voltageSysIdCommand() {
        return makeSysIdCommand(voltageSysIdRoutine);
    }

    public Command torqueCurrentSysIdCommand() {
        return makeSysIdCommand(torqueCurrentSysIdRoutine);
    }

}
