package frc.robot.subsystems.intake.slide;

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

public class IntakeSlide extends SubsystemBase {
    protected static final String LogKey = "/Intake/Slide";
    private static final double PositionToleranceRots = 0.01;
    private static final double VelocityToleranceRotsPerSec = 0.02;
    private static final double HardstopCurrentThresholdAmps = 30;

    private final Debouncer currentDebouncer = new Debouncer(0.3, Debouncer.DebounceType.kBoth);

    private final HardwareConstants.IntakeSlideConstants constants;

    private final IntakeSlideIO intakeSlideIO;
    private final IntakeSlideIOInputsAutoLogged inputs = new IntakeSlideIOInputsAutoLogged();

    private Goal desiredGoal = Goal.HOMING;
    private Goal currentGoal = desiredGoal;

    //TODO: Need to implement
    private ControlMode controlMode = ControlMode.HARD;

    public final Trigger atSlideSetpoint = new Trigger(this::atSlidePositionSetpoint);
    public final Trigger atSlideLowerLimit = new Trigger(this::atSlideLowerLimit);
    public final Trigger atSlideUpperLimit = new Trigger(this::atSlideUpperLimit);
    private final Trigger isAboveHomingCurrent = new Trigger(() -> currentDebouncer.calculate(Math.abs(
            inputs.masterTorqueCurrentAmps) >= HardstopCurrentThresholdAmps
            && Math.abs(inputs.followerTorqueCurrentAmps) >= HardstopCurrentThresholdAmps
    ));
    private final Trigger shouldUseSoftMode = new Trigger(() -> currentGoal == Goal.INTAKE && atSlideSetpoint.getAsBoolean())
            .onTrue(
                    Commands.runOnce(() -> this.controlMode = ControlMode.SOFT)
            ).onFalse(
                    Commands.runOnce(() -> this.controlMode = ControlMode.HARD)
            );

    private boolean isHomed = false;

    public enum Goal {
        HOMING(0),
        STOW(0),
        INTAKE(3.8);

        private final double slideGoalRotations;

        Goal(final double slideGoalRotations) {
            this.slideGoalRotations = slideGoalRotations;
        }

        public double getSlideGoalRotations() {
            return slideGoalRotations;
        }
    }

    public enum ControlMode {
        SOFT,
        HARD
    }

    public IntakeSlide(final Constants.RobotMode mode, final HardwareConstants.IntakeSlideConstants constants) {
        this.constants = constants;

        this.intakeSlideIO = switch (mode) {
            case REAL -> new IntakeSlideIOReal(constants);
            case SIM -> new IntakeSlideIOSim(constants);
            case REPLAY, DISABLED -> new IntakeSlideIO() {};
        };
    }

    @Override
    public void periodic() {
        final double IntakeSlidePeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeSlideIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            switch (controlMode) {
                case SOFT -> intakeSlideIO.holdSlidePosition(desiredGoal.getSlideGoalRotations());
                case HARD -> intakeSlideIO.toSlidePosition(desiredGoal.getSlideGoalRotations());
            }
            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/SlidePositionRots", desiredGoal.getSlideGoalRotations());

        Logger.recordOutput(LogKey + "/ControlMode", controlMode);
        Logger.recordOutput(LogKey + "/IsHomed", isHomed);

        Logger.recordOutput(LogKey + "/Triggers/AtPositionSetpoint", atSlidePositionSetpoint());
        Logger.recordOutput(LogKey + "/Triggers/AtSlideLowerLimit", atSlideLowerLimit());
        Logger.recordOutput(LogKey + "/Triggers/AtSlideUpperLimit", atSlideUpperLimit());
        Logger.recordOutput(LogKey + "/Triggers/IsAboveHomingCurrent", isAboveHomingCurrent);
        Logger.recordOutput(LogKey + "/Triggers/ShouldUseSoftMode", shouldUseSoftMode);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - IntakeSlidePeriodicUpdateStart)
        );
    }

    public Command home() {
        return Commands.sequence(
                Commands.runOnce(intakeSlideIO::home),
                Commands.waitUntil(isAboveHomingCurrent),
                Commands.runOnce(() -> {
                            intakeSlideIO.zeroMotors();
                            this.isHomed = true;
                        }
                )
        );
    }

    public boolean isHomed() {
        return isHomed;
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.STOW)
        ).withName("ToGoal: " + goal);
    }

    public Command setGoal(final Goal goal) {
        return runOnce(
                () -> setDesiredGoal(goal)
        ).withName("ToGoalL: " + goal);
    }

    private void setDesiredGoal(final Goal goal) {
        this.desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }

    public Rotation2d getIntakeSlidePositionRots() {
        return Rotation2d.fromRotations(inputs.averagePositionRots);
    }

    private boolean atSlidePositionSetpoint() {
        return currentGoal == desiredGoal
                && MathUtil.isNear(desiredGoal.getSlideGoalRotations(), inputs.masterPositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.masterVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }

    private boolean atSlideLowerLimit() {
        return inputs.masterPositionRots <= constants.lowerLimitRots();
    }

    private boolean atSlideUpperLimit() {
        return inputs.masterPositionRots >= constants.upperLimitRots();
    }
}
