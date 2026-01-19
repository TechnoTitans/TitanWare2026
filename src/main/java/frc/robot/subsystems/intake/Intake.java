package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;


public class Intake extends SubsystemBase {
    protected static final String LogKey = "Intake";
    private static final double PositionToleranceRots = 0.02;
    private static final double VelocityToleranceRotsPerSec = 0.02;

    private final HardwareConstants.IntakeConstants constants;

    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOW;
    private Goal currentGoal = desiredGoal;

    public final Trigger atSliderSetpoint = new Trigger(this::atSliderPositionSetpoint);
    public final Trigger atSliderLowerLimit = new Trigger(this::atSliderLowerLimit);
    public final Trigger atSliderUpperLimit = new Trigger(this::atSliderUpperLimit);

    public enum Goal {
        STOW(0, 0),
        INTAKE(1, 10),
        EJECT(1, -9);

        private final double sliderExtensionGoalMeters;
        private final double rollerVelocityGoalRotsPerSec;
        
        Goal(final double sliderExtensionGoalMeters, final double rollerVelocityGoalRotsPerSec) {
            this.sliderExtensionGoalMeters = sliderExtensionGoalMeters;
            this.rollerVelocityGoalRotsPerSec = rollerVelocityGoalRotsPerSec;
        }

        public double getSliderExtensionGoalMeters() {
            return sliderExtensionGoalMeters;
        }
        
        public double getSliderGoalRots(final double gearPitchCircumferenceMeters) {
            return this.sliderExtensionGoalMeters / gearPitchCircumferenceMeters;
        }
        
        public double getRollerVelocityGoalRotsPerSec() {
            return rollerVelocityGoalRotsPerSec;
        }
    }

    public Intake(final Constants.RobotMode mode, final HardwareConstants.IntakeConstants constants) {
        this.constants = constants;

        this.intakeIO = switch (mode) {
            case REAL -> new IntakeIOReal(constants);
            case SIM -> new IntakeIOSim(constants);
            case REPLAY, DISABLED -> new IntakeIO() {
            };
        };

        this.inputs = new IntakeIOInputsAutoLogged();
        this.intakeIO.config();

        intakeIO.toSliderPosition(desiredGoal.getSliderGoalRots(constants.gearPitchCircumferenceMeters()));
    }

    @Override
    public void periodic() {
        final double IntakePeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            intakeIO.toSliderPosition(desiredGoal.getSliderGoalRots(constants.gearPitchCircumferenceMeters()));
            intakeIO.toRollerVelocity(desiredGoal.getRollerVelocityGoalRotsPerSec());

            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());

        Logger.recordOutput(LogKey + "/CurrentGoal/RollerVelocity", currentGoal.getRollerVelocityGoalRotsPerSec());
        Logger.recordOutput(LogKey + "/DesiredGoal/RollerVelocity", desiredGoal.getRollerVelocityGoalRotsPerSec());

        Logger.recordOutput(LogKey + "/Roller/AtRollerVelocitySetpoint", atRollerVelocitySetpoint());

        Logger.recordOutput(LogKey + "/CurrentGoal/SliderPositionRots", currentGoal.getSliderGoalRots(constants.gearPitchCircumferenceMeters()));
        Logger.recordOutput(LogKey + "/DesiredGoal/SliderPositionRots", desiredGoal.getSliderGoalRots(constants.gearPitchCircumferenceMeters()));

        Logger.recordOutput(LogKey + "/Slider/AtPositionSetpoint", atSliderPositionSetpoint());
        Logger.recordOutput(LogKey + "/Slider/AtSliderLowerLimit", atSliderLowerLimit());
        Logger.recordOutput(LogKey + "/Slider/AtSliderUpperLimit", atSliderUpperLimit());
        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - IntakePeriodicUpdateStart)
        );
    }
    
    private boolean atSliderPositionSetpoint() {
        return currentGoal == desiredGoal
                && MathUtil.isNear(desiredGoal.getSliderGoalRots(constants.gearPitchCircumferenceMeters()), inputs.sliderPositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.sliderVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }

    public boolean atRollerVelocitySetpoint() {
        return currentGoal == desiredGoal
                && MathUtil.isNear(desiredGoal.rollerVelocityGoalRotsPerSec, inputs.rollerVelocityRotsPerSec, VelocityToleranceRotsPerSec);
    }

    private boolean atSliderLowerLimit() {
        return inputs.sliderPositionRots <= constants.lowerLimitRots();
    }

    private boolean atSliderUpperLimit() {
        return inputs.sliderPositionRots >= constants.upperLimitRots();
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                () -> setGoal(goal),
                () -> setGoal(Goal.STOW)
        ).withName("ToGoal");
    }

    public void setGoal(final Goal goal) {
        this.desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());

    }

    public Rotation2d getIntakeSliderPositionRots() {
        return Rotation2d.fromRotations(inputs.sliderPositionRots);
    }
}