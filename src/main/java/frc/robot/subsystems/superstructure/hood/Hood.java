package frc.robot.subsystems.superstructure.hood;

import edu.wpi.first.math.MathUtil;
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
    protected static final String LogKey = "Superstructure/Hood";
    private static final double PositionToleranceRots = 0.001;
    private static final double VelocityToleranceRotsPerSec = 0.001;
    private static final double HardstopCurrentThreshold = 1;

    private final HardwareConstants.HoodConstants constants;

    private final HoodIO hoodIO;
    private final HoodIOInputsAutoLogged inputs;

    private Goal previousGoal = Goal.STOW;
    private Goal desiredGoal = previousGoal;
    private Goal currentGoal = desiredGoal;

    public final Trigger atSetpoint = new Trigger(this::atHoodPositionSetpoint);
    public final Trigger atHoodLowerLimit = new Trigger(this::atHoodLowerLimit);
    public final Trigger atHoodUpperLimit = new Trigger(this::atHoodUpperLimit);

    private boolean isHomed;

    public enum Goal {
        STOW(0, false),
        CLIMB(0, false),
        TRACKING(0, true);

        private double hoodPositionGoalRots;
        private final boolean isDynamic;

        Goal(final double initialHoodPositionGoalRots, final boolean isDynamic) {
            this.hoodPositionGoalRots = initialHoodPositionGoalRots;
            this.isDynamic = isDynamic;
        }

        public void changeHoodPositionRots(final double desiredPositionRots) {
            if (isDynamic) {
                this.hoodPositionGoalRots = desiredPositionRots;
            }
        }

        public double getHoodPositionGoalRots() {
            return hoodPositionGoalRots;
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

        isHomed = Constants.CURRENT_MODE == Constants.RobotMode.SIM;

        this.inputs = new HoodIOInputsAutoLogged();

        this.hoodIO.config();
        this.hoodIO.toHoodPosition(desiredGoal.getHoodPositionGoalRots());
    }

    @Override
    public void periodic() {
        final double HoodPeriodicUpdateStart = Timer.getFPGATimestamp();
        hoodIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal.isDynamic) {
            hoodIO.toHoodContinuousPosition(desiredGoal.getHoodPositionGoalRots());
        }

        if (desiredGoal != currentGoal) {
            hoodIO.toHoodPosition(desiredGoal.getHoodPositionGoalRots());
            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/HoodPositionRots", desiredGoal.getHoodPositionGoalRots());
        Logger.recordOutput(LogKey + "/Triggers/AtPositionSetpoint", atHoodPositionSetpoint());
        Logger.recordOutput(LogKey + "/Triggers/AtHoodLowerLimit", atHoodLowerLimit());
        Logger.recordOutput(LogKey + "/Triggers/AtHoodUpperLimit", atHoodUpperLimit());
        Logger.recordOutput(LogKey + "/IsHomed", isHomed);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - HoodPeriodicUpdateStart)
        );
    }

    public Command home() {
        return Commands.sequence(
                Commands.runOnce(hoodIO::home),
                Commands.waitUntil(
                        () -> getCurrent() >= HardstopCurrentThreshold
                ),
                Commands.runOnce(() -> {
                                    hoodIO.zeroMotor();
                                    this.isHomed = true;
                                }
                        )
                        .finallyDo(() -> {
                            this.currentGoal = previousGoal;
                            this.previousGoal = Goal.TRACKING;
                        })
        );
    }

    public void setGoal(final Goal goal) {
        this.desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }

    public void updateDesiredHoodPosition(final double desiredHoodPosition) {
        this.desiredGoal.changeHoodPositionRots(desiredHoodPosition);
    }

    public Rotation2d getHoodPosition() {
        return Rotation2d.fromRotations(inputs.hoodPositionRots);
    }

    public boolean isHomed() {
        return isHomed;
    }

    private boolean atHoodPositionSetpoint() {
        return currentGoal == desiredGoal
                && MathUtil.isNear(desiredGoal.hoodPositionGoalRots, inputs.hoodPositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.hoodVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }

    private boolean atHoodLowerLimit() {
        return inputs.hoodPositionRots <= constants.hoodLowerLimitRots();
    }

    private boolean atHoodUpperLimit() {
        return inputs.hoodPositionRots >= constants.hoodUpperLimitRots();
    }

    private double getCurrent() {
        return inputs.hoodTorqueCurrentAmps;
    }
}
