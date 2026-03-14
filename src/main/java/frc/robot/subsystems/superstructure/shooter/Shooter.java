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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
    protected static final String LogKey = "Shooter";
    private static final double VelocityToleranceRotsPerSec = 3;

    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs;

    private SysIdRoutine flywheelVoltageSysIdRoutine;
    private SysIdRoutine flywheelTorqueCurrentSysIdRoutine;

    private Goal desiredGoal = Goal.TRACKING;
    private Goal currentGoal = desiredGoal;

    public final Trigger atSetpoint = new Trigger(this::atSetpoint);

    public enum Goal {
        IDLE(20, false),
        TRACKING(0, true),
        BASIC(35, false);

        private double velocitySetpointRotsPerSec;
        private final boolean isDynamic;

        Goal(final double velocitySetpointRotsPerSec, final boolean isDynamic) {
            this.velocitySetpointRotsPerSec = velocitySetpointRotsPerSec;
            this.isDynamic = isDynamic;
        }

        public void changeShooterVelocityGoal(final double desiredShooterVelocity) {
            if (isDynamic) {
                this.velocitySetpointRotsPerSec = desiredShooterVelocity;
            }
        }
    }

    public Shooter(final Constants.RobotMode mode, final HardwareConstants.ShooterConstants constants) {
        this.shooterIO = switch (mode) {
            case REAL -> new ShooterIORealBangBang(constants);
            case SIM -> new ShooterIOSim(constants);
            case REPLAY, DISABLED -> new ShooterIO() {};
        };

        this.inputs = new ShooterIOInputsAutoLogged();

        this.shooterIO.config();
        this.shooterIO.toFlywheelVelocity(desiredGoal.velocitySetpointRotsPerSec);
    }

    @Override
    public void periodic() {
        final double shooterPeriodicUpdateStart = Timer.getFPGATimestamp();

        shooterIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            currentGoal = desiredGoal;
            shooterIO.toFlywheelVelocity(desiredGoal.velocitySetpointRotsPerSec);
        } else if (desiredGoal.isDynamic) {
            shooterIO.toFlywheelVelocity(desiredGoal.velocitySetpointRotsPerSec);
        }

        this.flywheelVoltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(2).per(Second),
                Volts.of(10),
                Seconds.of(10),
                shooterIO::toFlywheelVoltage
        );
        this.flywheelTorqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(4).per(Second),
                Amp.of(40),
                Seconds.of(10),
                shooterIO::toFlywheelTorqueCurrent
        );

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/VelocitySetpointRotsPerSec", desiredGoal.velocitySetpointRotsPerSec);

        Logger.recordOutput(LogKey + "/Triggers/AtSetpoint", atSetpoint());
        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - shooterPeriodicUpdateStart)
        );
    }

    public Command setGoalCommand(final Goal goal) {
        return Commands.runOnce(() -> setGoal(goal));
    }

    public double getVelocityRotsPerSec() {
        return inputs.masterVelocityRotsPerSec;
    }

    public void setGoal(final Goal goal) {
        desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", goal.toString());
    }

    public void updateVelocitySetpoint(final double desiredShooterVelocity) {
        desiredGoal.changeShooterVelocityGoal(desiredShooterVelocity);
    }

    private boolean atSetpoint() {
        return currentGoal == desiredGoal
                && MathUtil.isNear(
                desiredGoal.velocitySetpointRotsPerSec,
                inputs.masterVelocityRotsPerSec,
                VelocityToleranceRotsPerSec
        );
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
        SysIdRoutine sysIdRoutine = new SysIdRoutine(
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
        return sysIdRoutine;
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

    public Command flywheelVoltageSysIdCommand() {
        return makeFlywheelSysIdCommand(flywheelVoltageSysIdRoutine);
    }

    public Command flywheelTorqueCurrentSysIdCommand() {
        return makeFlywheelSysIdCommand(flywheelTorqueCurrentSysIdRoutine);
    }
}
