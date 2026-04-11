package frc.robot.subsystems.indexer.feeder;

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

public class Feeder extends SubsystemExt {
    protected static final String LogKey = "Feeder";

    public enum Goal {
        BACK_OUT(-20),
        OFF(0),
        FEED(40);

        private final double velocityRotsPerSec;

        Goal(final double velocityRotsPerSec) {
            this.velocityRotsPerSec = velocityRotsPerSec;
        }
    }

    private final FeederIO feederIO;
    private final FeederIO.FeederIOInputs inputs = new FeederIO.FeederIOInputs();

    private Goal desiredGoal = Goal.OFF;
    private double velocitySetpoint = 0.0;

    private final SysIdRoutine wheelTorqueCurrentSysIdRoutine;

    public Feeder(final Constants.RobotMode mode, final HardwareConstants.FeederConstants constants) {
        this.feederIO = switch (mode) {
            case REAL -> new FeederIOReal(constants);
            case SIM -> new FeederIOSim(constants);
            case REPLAY, DISABLED -> new FeederIO() {};
        };

        this.wheelTorqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(4).per(Second),
                Amp.of(40),
                Seconds.of(10),
                feederIO::toWheelTorqueCurrent
        );
    }

    @Override
    public void periodic() {
        final double feederPeriodicFPGATime = Timer.getFPGATimestamp();

        feederIO.updateInputs(inputs);

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/VelocitySetpoint", velocitySetpoint);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - feederPeriodicFPGATime)
        );
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.OFF)
        ).withName("ToGoal: " + goal);
    }

    public Command wheelTorqueCurrentSysIdCommand() {
        return makeWheelSysIdCommand(wheelTorqueCurrentSysIdRoutine);
    }

    public double getSupplyCurrent() {
        return inputs.wheelTorqueCurrentAmps;
    }

    public boolean isTOFDetected() {
        return inputs.tofDetected;
    }

    public void setTOFDetected(final boolean isDetected) {
        feederIO.setTOFDetected(isDetected);
    }

    private void setDesiredGoal(final Goal goal) {
        desiredGoal = goal;
        setDesiredVelocity(goal.velocityRotsPerSec);
    }

    private void setDesiredVelocity(final double velocityRotsPerSec) {
        velocitySetpoint = velocityRotsPerSec;
        feederIO.toWheelVelocity(velocityRotsPerSec);
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