package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;


public class Intake extends SubsystemBase {
    protected static final String LogKey = "Intake";
    private static final double PositionToleranceRots = 0.002;
    private static final double VelocityToleranceRotsPerSec = 0.002;

    private final HardwareConstants.IntakeConstants constants;

    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOW;
    private Goal currentGoal = desiredGoal;

    public final Trigger atSetpoint = new Trigger(this::atSliderPositionSetpoint);
    public final Trigger atSliderLowerLimit = new Trigger(this::atSliderLowerLimit);
    public final Trigger atSliderUpperLimit = new Trigger(this::atSliderUpperLimit);

    public enum Goal {
        STOW(0, 0),
        INTAKE(0.5, 10),
        EJECT(0.5, -9);

        private final double sliderExtensionGoalMeters;
        private final double rollerVelocityGoal;
        
        Goal(final double sliderExtensionGoalMeters, final double rollerVelocityGoal) {
            this.sliderExtensionGoalMeters = sliderExtensionGoalMeters;
            this.rollerVelocityGoal = rollerVelocityGoal;
        }

        public double getSliderExtensionGoalMeters() {
            return sliderExtensionGoalMeters;
        }
        
        public double getSliderGoalRots(final double gearPitchMeters) {
            return this.sliderExtensionGoalMeters / gearPitchMeters;
        }
        
        public double getRollerVelocityGoal() {
            return rollerVelocityGoal;
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
    }

    @Override
    public void periodic() {
        final double IntakePeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(LogKey + "GoalChangeNeeded", desiredGoal != currentGoal);

        if (desiredGoal != currentGoal) {
            intakeIO.toSliderPosition(desiredGoal.getSliderGoalRots(constants.gearPitchCircumferenceMeters()));
            intakeIO.toRollerVelocity(desiredGoal.getRollerVelocityGoal());

            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());

        Logger.recordOutput(
               LogKey + "/CurrentGoal/RollerVelocity", currentGoal.getRollerVelocityGoal())
        ;

        Logger.recordOutput(
                LogKey + "/DesiredGoal/RollerVelocity", desiredGoal.getRollerVelocityGoal())
        ;

        Logger.recordOutput(LogKey + "/CurrentGoal/SliderGoal", currentGoal.getSliderGoalRots(constants.gearPitchCircumferenceMeters()));
        Logger.recordOutput(LogKey + "/DesiredGoal/SliderGoal", desiredGoal.getSliderGoalRots(constants.gearPitchCircumferenceMeters()));

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
                && MathUtil.isNear(desiredGoal.getSliderExtensionGoalMeters(), inputs.sliderPositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.sliderVelocityRotsPerSec, VelocityToleranceRotsPerSec);
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
}