package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Turret extends SubsystemBase {
    protected static final String LogKey = "Turret";

    private static final double PositionToleranceRots = 0.02;
    private static final double VelocityToleranceRotsPerSec = 0.02;

    private final HardwareConstants.TurretConstants constants;
    private final DoubleSupplier robotAngularVelocitySupplier;

    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.TRACKING;
    private Goal currentGoal = desiredGoal;

    public final Trigger atSetpoint = new Trigger(this::atSetpoint);

    public enum Goal {
        TRACKING(0, true);

        private double positionSetpointRots;
        private final boolean isDynamic;

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
            final DoubleSupplier robotAngularVelocitySupplier
    ) {
        this.constants = constants;
        this.turretIO = switch (mode) {
            case REAL -> new TurretIOReal(constants);
            case SIM -> new TurretIOSim(constants);
            case REPLAY, DISABLED -> new TurretIO() {};
        };

        this.robotAngularVelocitySupplier = robotAngularVelocitySupplier;

        this.inputs = new TurretIOInputsAutoLogged();
        this.turretIO.config();

        turretIO.setPosition(0);
    }

    @Override
    public void periodic() {
        final double turretPeriodicUpdateStart = Timer.getFPGATimestamp();

        turretIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            currentGoal = desiredGoal;
        }

        if (desiredGoal.isDynamic) {
            turretIO.toTurretContinuousPosition(
                    desiredGoal.positionSetpointRots,
                    -Units.radiansToRotations(robotAngularVelocitySupplier.getAsDouble())
            );
        } else {
            turretIO.toTurretContinuousPosition(desiredGoal.positionSetpointRots, 0);
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
                && MathUtil.isNear(robotAngularVelocitySupplier.getAsDouble(), inputs.turretVelocityRotsPerSec, VelocityToleranceRotsPerSec);
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
}