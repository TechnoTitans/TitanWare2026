package frc.robot.subsystems.superstructure.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.position.ChineseRemainder;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Turret extends SubsystemBase {
    protected static final String LogKey = "Turret";

    private static final double PositionToleranceRots = 0.001;
    private static final double VelocityToleranceRotsPerSec = 0.01;
    public static final double WRAP_THRESHOLD = 0.3;

    private final HardwareConstants.TurretConstants constants;
    private final DoubleSupplier robotAngularVelocitySupplier;

    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.TRACKING;
    private Goal currentGoal = desiredGoal;

    public final Trigger atSetpoint = new Trigger(this::atSetpoint);

    public enum Goal {
        CLIMB(0, false),
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

        this.inputs = new TurretIOInputsAutoLogged();

        this.turretIO.config();
        this.turretIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        this.robotAngularVelocitySupplier = robotAngularVelocitySupplier;

        final Rotation2d absolutePosition = ChineseRemainder.findAbsolutePosition(
                constants.turretTooth(),
                inputs.smallEncoderPositionRots,
                constants.smallEncoderTooth(),
                inputs.largeEncoderPositionRots,
                constants.largeEncoderTooth()
        );
        turretIO.seedTurretPosition(absolutePosition);

        Logger.recordOutput(LogKey + "/CRTResult", absolutePosition);
    }

    @Override
    public void periodic() {
        final double turretPeriodicUpdateStart = Timer.getFPGATimestamp();

        turretIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal.isDynamic) {
            if (Math.abs(inputs.turretPositionRots - desiredGoal.positionSetpointRots) > WRAP_THRESHOLD) {
                turretIO.toTurretPosition(desiredGoal.positionSetpointRots);
            } else {
                turretIO.toTurretContinuousPosition(desiredGoal.positionSetpointRots,
                        Units.radiansToRotations(-robotAngularVelocitySupplier.getAsDouble())
                );
            }
        }

        if (desiredGoal != currentGoal) {
            turretIO.toTurretPosition(desiredGoal.positionSetpointRots);
            currentGoal = desiredGoal;
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

    public void setGoal(final Goal goal) {
        desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal);
        Logger.recordOutput(LogKey + "/DesiredGoal", goal);
    }

    public void updatePositionSetpoint(final double desiredTurretPosition) {
        desiredGoal.changeTurretPositionRots(desiredTurretPosition);
    }

    public Rotation2d getTurretPosition() {
        return Rotation2d.fromRotations(inputs.turretPositionRots);
    }

    private boolean atSetpoint() {
        return currentGoal == desiredGoal
                && MathUtil.isNear(desiredGoal.positionSetpointRots, inputs.turretPositionRots, PositionToleranceRots)
                && MathUtil.isNear(robotAngularVelocitySupplier.getAsDouble(), inputs.turretVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }

    private boolean atLowerLimit() {
        return inputs.turretPositionRots <= constants.reverseLimitRots();
    }

    private boolean atUpperLimit() {
        return inputs.turretPositionRots >= constants.forwardLimitRots();
    }
}