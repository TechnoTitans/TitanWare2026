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
    protected static final String LogKey = "Superstructure/Turret";

    private static final double PositionToleranceRots = 0.02;
    private static final double VelocityToleranceRotsPerSec = 0.02;

    private final HardwareConstants.TurretConstants constants;
    private final DoubleSupplier robotAngularVelocitySupplier;

    private final TurretIO turretIO;
    private final TurretIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.TRACKING;
    private Goal currentGoal = desiredGoal;

    public final Trigger atSetpoint = new Trigger(this::atTurretPositionSetpoint);
    public final Trigger atTurretLowerLimit = new Trigger(this::atTurretLowerLimit);
    public final Trigger atTurretUpperLimit = new Trigger(this::atTurretUpperLimit);

    public enum Goal {
        CLIMB(0, false),
        TRACKING(0, true);

        private double turretPositionGoalRots;
        private final boolean isDynamic;

        Goal(final double turretPositionGoalRots, final boolean isDynamic) {
            this.turretPositionGoalRots = turretPositionGoalRots;
            this.isDynamic = isDynamic;
        }

        public void changeTurretPositionRots(final double desiredTurretPositionRots) {
            if (isDynamic) {
                this.turretPositionGoalRots = desiredTurretPositionRots;
            }
        }

        public double getTurretPositionGoalRots() {
            return turretPositionGoalRots;
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

        this.robotAngularVelocitySupplier = robotAngularVelocitySupplier;

        turretIO.updateInputs(inputs);

        try {
            final double absolutePosition = ChineseRemainder.getAbsolutePosition(
                    constants.leftEncoderGearing()/constants.turretTooth(),
                    Units.rotationsToDegrees(inputs.leftPositionRots),
                    constants.rightEncoderGearing()/constants.turretTooth(),
                    Units.rotationsToDegrees(inputs.rightPositionRots),
                    constants.turretTooth()
            ) % 1.0;

            turretIO.setTurretPosition(absolutePosition - 0.5);
        } catch (RuntimeException e) {
            turretIO.setTurretPosition(0.0);
        }
    }

    @Override
    public void periodic() {
        final double TurretPeriodicUpdateStart = Timer.getFPGATimestamp();

        turretIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        //TODO: Simplify logic
        if (desiredGoal != currentGoal) {
            turretIO.toTurretPosition(desiredGoal.getTurretPositionGoalRots());
            this.currentGoal = desiredGoal;
        } else if (desiredGoal.isDynamic) {
            if (Math.abs(inputs.turretPositionRots - desiredGoal.getTurretPositionGoalRots()) > 0.3) {
                turretIO.toTurretPosition(desiredGoal.getTurretPositionGoalRots());
            } else {
                turretIO.toTurretContinuousPosition(desiredGoal.getTurretPositionGoalRots(), Units.radiansToRotations(robotAngularVelocitySupplier.getAsDouble()));
            }
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/TurretPositionRots", desiredGoal.getTurretPositionGoalRots());

        Logger.recordOutput(LogKey + "/Triggers/AtPositionSetpoint", atTurretPositionSetpoint());
        Logger.recordOutput(LogKey + "/Triggers/AtLowerLimit", atTurretLowerLimit());
        Logger.recordOutput(LogKey + "/Triggers/AtUpperLimit", atTurretUpperLimit());
        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - TurretPeriodicUpdateStart)
        );
    }

    public void setGoal(final Goal desiredGoal) {
        this.desiredGoal = desiredGoal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal);
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
    }

    public void updatePositionSetpoint(final double desiredTurretPosition) {
        this.desiredGoal.changeTurretPositionRots(desiredTurretPosition);
    }

    public Rotation2d getTurretPosition() {
        return Rotation2d.fromRotations(inputs.turretPositionRots);
    }

    private boolean atTurretPositionSetpoint() {
        return currentGoal == desiredGoal
                && MathUtil.isNear(desiredGoal.getTurretPositionGoalRots(), inputs.turretPositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.turretVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }

    private boolean atTurretLowerLimit() {
        return inputs.turretPositionRots <= constants.lowerLimitRots();
    }

    private boolean atTurretUpperLimit() {
        return inputs.turretPositionRots >= constants.upperLimitRots();
    }

    public TurretIO getTurretIO() { return turretIO; }
}