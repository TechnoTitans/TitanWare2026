package frc.robot.subsystems.indexer.spindexer;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.ext.SubsystemExt;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;

public class Spindexer extends SubsystemExt {
    protected static final String LogKey = "Spindexer";

    public enum Goal {
        BACK_OUT(-6),
        STOP(0),
        FEED(9);

        private final double velocityRotsPerSec;

        Goal(final double velocityRotsPerSec) {
            this.velocityRotsPerSec = velocityRotsPerSec;
        }
    }

    private final SpindexerIO spindexerIO;
    private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    private Goal desiredGoal = Goal.STOP;
    private double velocitySetpoint = 0.0;

    private final SysIdRoutine wheelTorqueCurrentSysIdRoutine;

    public Spindexer(final Constants.RobotMode mode, final HardwareConstants.SpindexerConstants constants) {
        this.spindexerIO = switch (mode) {
            case REAL -> new SpindexerIOReal(constants);
            case SIM -> new SpindexerIOSim(constants);
            case REPLAY, DISABLED -> new SpindexerIO() {};
        };

        this.wheelTorqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(4).per(Second),
                Amp.of(40),
                Seconds.of(10),
                spindexerIO::toWheelTorqueCurrent
        );
    }

    @Override
    public void periodic() {
        final double spindexerPeriodicFPGATime = Timer.getFPGATimestamp();

        spindexerIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/VelocitySetpoint", velocitySetpoint);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - spindexerPeriodicFPGATime)
        );
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.STOP)
        ).withName("ToGoal: " + goal);
    }

    public Command setGoal(final Goal goal) {
        return Commands.runOnce(() -> setDesiredGoal(goal))
                .withName("SetGoal:" + goal);
    }

    public Command wheelTorqueCurrentSysIdCommand() {
        return makeWheelSysIdCommand(wheelTorqueCurrentSysIdRoutine);
    }

    private void setDesiredGoal(final Goal goal) {
        desiredGoal = goal;
        setDesiredVelocity(goal.velocityRotsPerSec);
    }

    private void setDesiredVelocity(final double velocityRotsPerSec) {
        velocitySetpoint = velocityRotsPerSec;
        spindexerIO.toWheelVelocity(velocityRotsPerSec);
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

    private Command makeWheelSysIdCommand(final SysIdRoutine sysIdRoutine) {
        return Commands.sequence(
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(5),
                Commands.waitUntil(() -> Math.abs(inputs.wheelVelocityRotsPerSec) < 0.5),
                Commands.waitSeconds(1),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(5),
                Commands.waitUntil(() -> Math.abs(inputs.wheelVelocityRotsPerSec) < 0.5),
                Commands.waitSeconds(1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(5),
                Commands.waitUntil(() -> Math.abs(inputs.wheelVelocityRotsPerSec) < 0.5),
                Commands.waitSeconds(1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(5)
        );
    }
}
