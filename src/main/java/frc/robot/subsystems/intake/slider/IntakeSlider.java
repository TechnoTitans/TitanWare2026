package frc.robot.subsystems.intake.slider;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

public class IntakeSlider extends SubsystemBase {
    protected static final String LogKey = "IntakeSlider";
    private static final double PositionToleranceRots = 0.002;
    private static final double VelocityToleranceRotsPerSec = 0.002;

    private final HardwareConstants.IntakeSliderConstants constants;

    private final IntakeSliderIO intakeSliderIO;
    private final IntakeArmIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOW;
    private Goal currentGoal = desiredGoal;

    public final Trigger atSetpoint = new Trigger(this::atPivotPositionSetpoint);

    public enum Goal {
        STOW(0),
        INTAKE(1);

        private final double positionGoalMeters;
        Goal(final double positionGoalMeters) {
            this.positionGoalMeters = positionGoalMeters;
        }

        public double getPositionGoalMeters() {
            return positionGoalMeters;
        }

        public double getPositionGoalRots(final double gearPitchCircumference) {
            return getPositionGoalMeters() / gearPitchCircumference;
        }
    }

    public IntakeSlider(final Constants.RobotMode mode, final HardwareConstants.IntakeSliderConstants constants) {
        this.constants = constants;
        this.intakeSliderIO = switch(mode) {
            case REAL -> new IntakeSliderIOReal(constants);
            case SIM -> new IntakeSliderIOSim(constants);
            case DISABLED, REPLAY -> new IntakeSliderIO() {};
        };

        this.inputs = new IntakeArmIOInputsAutoLogged();

        this.intakeSliderIO.config();
        zeroSlider();

        this.intakeSliderIO.toPosition(desiredGoal.getPositionGoalRots(constants.gearPitchCircumference()));
    }

    @Override
    public void periodic() {
        final double intakeSliderPeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeSliderIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            intakeSliderIO.toPosition(desiredGoal.positionGoalMeters);

            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal/SliderPositionMeters", desiredGoal.getPositionGoalMeters());
        Logger.recordOutput(LogKey + "/AtPositionSetpoint", atPivotPositionSetpoint());
        Logger.recordOutput(LogKey + "/AtPivotLowerLimit", atPivotLowerLimit());
        Logger.recordOutput(LogKey + "/AtPivotUpperLimit", atPivotUpperLimit());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - intakeSliderPeriodicUpdateStart)
        );
    }

    private boolean atPivotPositionSetpoint() {
        return desiredGoal == currentGoal
                && MathUtil.isNear(desiredGoal.getPositionGoalRots(constants.gearPitchCircumference()), inputs.sliderPositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.sliderVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }

    private boolean atPivotLowerLimit() {
        return inputs.pivotPositionRots <= constants.lowerLimitRots();
    }

    private boolean atPivotUpperLimit() {
        return inputs.pivotPositionRots >= constants.upperLimitRots();
    }

    public void setGoal(final Goal goal) {
        this.desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }

    private void zeroSlider() {
        intakeSliderIO.zeroPosition();
    }
}

