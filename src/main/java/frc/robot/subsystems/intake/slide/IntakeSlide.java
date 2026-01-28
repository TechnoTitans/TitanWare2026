package frc.robot.subsystems.intake.slide;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;

public class IntakeSlide extends SubsystemBase {
    protected static final String LogKey = "/Intake/Slide";
    private static final double PositionToleranceRots = 0.02;
    private static final double VelocityToleranceRotsPerSec = 0.02;

    private final HardwareConstants.IntakeSlideConstants constants;

    private final IntakeSlideIO intakeSlideIO;
    private final IntakeSlideIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOW;
    private Goal currentGoal = desiredGoal;

    public final Trigger atSlideSetpoint = new Trigger(this::atSlidePositionSetpoint);
    public final Trigger atSlideLowerLimit = new Trigger(this::atSlideLowerLimit);
    public final Trigger atSlideUpperLimit = new Trigger(this::atSlideUpperLimit);

    public enum Goal {
        STOW(0),
        INTAKE(1),
        EJECT(1),
        SHOOTING(0);

        private final double slideExtensionGoalMeters;

        Goal(final double slideExtensionGoalMeters) {
            this.slideExtensionGoalMeters = slideExtensionGoalMeters;
        }

        public double getSlideExtensionGoalMeters() {
            return slideExtensionGoalMeters;
        }

        public double getSlideGoalRots(final double gearPitchCircumferenceMeters) {
            return this.slideExtensionGoalMeters / gearPitchCircumferenceMeters;
        }
    }

    public IntakeSlide(final Constants.RobotMode mode, final HardwareConstants.IntakeSlideConstants constants) {
        this.constants = constants;

        this.intakeSlideIO = switch (mode) {
            case REAL -> new IntakeSlideIOReal(constants);
            case SIM -> new IntakeSlideIOSim(constants);
            case REPLAY, DISABLED -> new IntakeSlideIO() {};
        };

        this.inputs = new IntakeSlideIOInputsAutoLogged();
        this.intakeSlideIO.config();

        intakeSlideIO.toSlidePosition(desiredGoal.getSlideGoalRots(constants.gearPitchCircumferenceMeters()));
    }

    @Override
    public void periodic() {
        final double IntakeSlidePeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeSlideIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            if (desiredGoal == Goal.SHOOTING){
                intakeSlideIO.toSlidePositionUnprofiled(desiredGoal.getSlideGoalRots(constants.gearPitchCircumferenceMeters()), 0.01);
            }else {
                intakeSlideIO.toSlidePosition(desiredGoal.getSlideGoalRots(constants.gearPitchCircumferenceMeters()));
            }
            intakeSlideIO.toSlidePosition(desiredGoal.getSlideGoalRots(constants.gearPitchCircumferenceMeters()));
            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());

        Logger.recordOutput(LogKey + "/CurrentGoal/SlidePositionRots", currentGoal.getSlideGoalRots(constants.gearPitchCircumferenceMeters()));
        Logger.recordOutput(LogKey + "/DesiredGoal/SlidePositionRots", desiredGoal.getSlideGoalRots(constants.gearPitchCircumferenceMeters()));

        Logger.recordOutput(LogKey + "/Slide/AtPositionSetpoint", atSlidePositionSetpoint());
        Logger.recordOutput(LogKey + "/Slide/AtSlideLowerLimit", atSlideLowerLimit());
        Logger.recordOutput(LogKey + "/Slide/AtSlideUpperLimit", atSlideUpperLimit());
        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - IntakeSlidePeriodicUpdateStart)
        );
    }

    private boolean atSlidePositionSetpoint() {
        return currentGoal == desiredGoal
                && MathUtil.isNear(desiredGoal.getSlideGoalRots(constants.gearPitchCircumferenceMeters()), inputs.masterPositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.masterVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }

    private boolean atSlideLowerLimit() {
        return inputs.masterPositionRots <= constants.lowerLimitRots();
    }

    private boolean atSlideUpperLimit() {
        return inputs.masterPositionRots >= constants.upperLimitRots();
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.STOW)
        ).withName("ToGoal");
    }

    public Command setGoal(final Goal goal) {
        return runOnce(
                () -> setDesiredGoal(goal)
        ).withName("ToGoal");
    }

    private void setDesiredGoal(final Goal goal) {
        this.desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }

    public Rotation2d getIntakeSlidePositionRots() {
        return Rotation2d.fromRotations(inputs.masterPositionRots);
    }
}
