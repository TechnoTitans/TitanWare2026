package frc.robot.subsystems.intake.rollers;

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

public class IntakeRollers extends SubsystemExt {
    protected static final String LogKey = "IntakeRollers";

    public enum Goal {
        UNSTUCK(-40),
        OFF(0),
        FEED_PULSE(15),
        INTAKE(40);

        private final double velocityRotsPerSec;

        Goal(final double velocityRotsPerSec) {
            this.velocityRotsPerSec = velocityRotsPerSec;
        }
    }

    private final IntakeRollersIO intakeRollersIO;
    private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

    private Goal desiredGoal = Goal.OFF;
    private double velocitySetpoint = 0.0;

    private final SysIdRoutine rollersTorqueCurrentSysIdRoutine;

    public IntakeRollers(final Constants.RobotMode mode, final HardwareConstants.IntakeRollerConstants constants) {
        this.intakeRollersIO = switch (mode) {
            case REAL -> new IntakeRollersIOReal(constants);
            case SIM -> new IntakeRollersIOSim(constants);
            case REPLAY, DISABLED -> new IntakeRollersIO() {};
        };

        this.rollersTorqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(4).per(Second),
                Amp.of(40),
                Seconds.of(10),
                intakeRollersIO::toRollersTorqueCurrent
        );
    }

    @Override
    public void periodic() {
        final double intakeRollersPeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeRollersIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/VelocitySetpoint", velocitySetpoint);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - intakeRollersPeriodicUpdateStart)
        );
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.OFF)
        ).withName("ToGoal: " + goal);
    }

    public Command setGoal(final Goal goal) {
        return runOnce(() -> setDesiredGoal(goal))
                .withName("SetGoal: " + goal);
    }

    public Command rollersTorqueCurrentSysIdCommand() {
        return makeRollersSysIdCommand(rollersTorqueCurrentSysIdRoutine);
    }

    public double getSupplyCurrent() {
        return inputs.masterTorqueCurrentAmps + inputs.followerTorqueCurrentAmps;
    }

    private void setDesiredGoal(final Goal goal) {
        desiredGoal = goal;
        setDesiredVelocity(goal.velocityRotsPerSec);
    }

    private void setDesiredVelocity(final double velocityRotsPerSec) {
        velocitySetpoint = velocityRotsPerSec;
        intakeRollersIO.toRollersVelocity(velocityRotsPerSec);
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

    private Command makeRollersSysIdCommand(final SysIdRoutine sysIdRoutine) {
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
