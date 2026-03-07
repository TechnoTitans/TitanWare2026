package frc.robot.subsystems.superstructure.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
    protected static final String LogKey = "Hood";
    private static final double PositionToleranceRots = 0.05;
    private static final double VelocityToleranceRotsPerSec = 0.01;
    private static final double ZeroingCurrentThresholdAmps = 2;

    private final HardwareConstants.HoodConstants constants;

    private final HoodIO hoodIO;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    private Goal desiredGoal = Goal.HOMING;
    private Goal currentGoal = desiredGoal;

    public final Trigger atSetpoint = new Trigger(this::atSetpoint);
    private final Trigger isAboveHomingCurrent = new Trigger(
            () -> Math.abs(inputs.hoodTorqueCurrentAmps) >= ZeroingCurrentThresholdAmps
    ).debounce(0.15, Debouncer.DebounceType.kRising);

    private boolean isHomed = false;

    public enum Goal {
        HOMING(0, false),
        STOW(0, false),
        CLIMB(0, false),
        TRACKING(0, true);

        private double positionSetpointRots;
        private final boolean isDynamic;

        Goal(final double positionSetpointRots, final boolean isDynamic) {
            this.positionSetpointRots = positionSetpointRots;
            this.isDynamic = isDynamic;
        }

        public void changeHoodPositionRots(final double desiredPositionRots) {
            if (isDynamic) {
                this.positionSetpointRots = desiredPositionRots;
            }
        }
    }

    public Hood(final Constants.RobotMode mode, final HardwareConstants.HoodConstants constants) {
        this.constants = constants;
        this.hoodIO = switch (mode) {
            case REAL -> new HoodIOReal(constants);
            case SIM -> new HoodIOSim(constants);
            case DISABLED, REPLAY -> new HoodIO() {
            };
        };

        hoodIO.config();
    }

    @Override
    public void periodic() {
        final double hoodPeriodicUpdateStart = Timer.getFPGATimestamp();
        hoodIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal.isDynamic) {
            hoodIO.toHoodContinuousPosition(desiredGoal.positionSetpointRots);
        }

        if (desiredGoal != currentGoal) {
            hoodIO.toHoodPosition(desiredGoal.positionSetpointRots);
            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/PositionSetpointRots", desiredGoal.positionSetpointRots);

        Logger.recordOutput(LogKey + "/IsHomed", isHomed);

        Logger.recordOutput(LogKey + "/Triggers/AtSetpoint", atSetpoint());
        Logger.recordOutput(LogKey + "/Triggers/AtHoodLowerLimit", atLowerLimit());
        Logger.recordOutput(LogKey + "/Triggers/AtHoodUpperLimit", atUpperLimit());
        Logger.recordOutput(LogKey + "/Triggers/IsAboveHomingCurrent", isAboveHomingCurrent);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - hoodPeriodicUpdateStart)
        );
    }

    public Command home() {
        return Commands.sequence(
                Commands.runOnce(() -> hoodIO.toHoodTorqueCurrent(-10)),
                Commands.waitUntil(isAboveHomingCurrent),
                Commands.runOnce(() -> {
                        hoodIO.zeroMotor();
                        this.isHomed = true;
                    }
                )
        );
    }

    public void setDesiredGoal(final Goal goal) {
        desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }

    public void updateDesiredHoodPosition(final double desiredHoodPosition) {
        desiredGoal.changeHoodPositionRots(desiredHoodPosition);
    }

    public Rotation2d getHoodPosition() {
        return Rotation2d.fromRotations(inputs.hoodPositionRots);
    }

    public boolean isHomed() {
        return isHomed;
    }

    private boolean atSetpoint() {
        return currentGoal == desiredGoal
                && MathUtil.isNear(desiredGoal.positionSetpointRots, inputs.hoodPositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.hoodVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }

    private boolean atLowerLimit() {
        return inputs.hoodPositionRots <= constants.hoodLowerLimitRots();
    }

    private boolean atUpperLimit() {
        return inputs.hoodPositionRots >= constants.hoodUpperLimitRots();
    }
}
