package frc.robot.subsystems.intake.slide;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.commands.LoggedTrigger;
import frc.robot.utils.commands.SubsystemExt;
import frc.robot.utils.control.DeltaTime;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Objects;

public class IntakeSlide extends SubsystemExt {
    protected static final String LogKey = "IntakeSlide";
    protected static final double PositionToleranceRots = 0.1; //TODO: Check
    private static final double VelocityToleranceRotsPerSec = 0.02;

    public enum ProfileType {
        EXPO,
        WITH_VELOCITY
    }

    public static class GoalBehavior {
        public final ProfileType type;
        public final double velocity;
        public final double acceleration;

        public final TrapezoidProfile profile;

        private GoalBehavior(final ProfileType type, final double velocity, final double acceleration) {
            this.type = type;
            this.velocity = velocity;
            this.acceleration = acceleration;

            this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(velocity, acceleration));
        }

        public static GoalBehavior expo() {
            return new GoalBehavior(ProfileType.EXPO, 0, 0);
        }

        public static GoalBehavior withVelocity(final double velocity, final double acceleration) {
            return new GoalBehavior(ProfileType.WITH_VELOCITY, velocity, acceleration);
        }
    }

    public enum Goal {
        STOW(0, GoalBehavior.expo()),
        INTAKE(3.8, GoalBehavior.expo()),
        STOW_FEED(0, GoalBehavior.withVelocity(1.2, 0.75));

        public final double positionRots;
        public final GoalBehavior behavior;

        Goal(final double positionRots, final GoalBehavior behavior) {
            this.positionRots = positionRots;
            this.behavior = behavior;
        }
    }

    private enum InternalGoal {
        NONE,
        STOW(Goal.STOW),
        INTAKE(Goal.INTAKE),
        STOW_FEED(Goal.STOW_FEED),
        HOLD_POSITION;

        public static final HashMap<Goal, InternalGoal> GoalToInternal = new HashMap<>();
        static {
            for (final InternalGoal goal : InternalGoal.values()) {
                if (goal.goal != null) {
                    GoalToInternal.put(goal.goal, goal);
                }
            }
        }

        public static InternalGoal fromGoal(final Goal goal) {
            return Objects.requireNonNull(GoalToInternal.get(goal));
        }

        public final Goal goal;

        InternalGoal(final Goal goal) {
            this.goal = goal;
        }

        InternalGoal() {
            this(null);
        }
    }

    public enum HoldMode {
        STIFF,
        SQUISHY
    }

    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final IntakeSlideIO intakeSlideIO;
    private final IntakeSlideIOInputsAutoLogged inputs;

    private final DeltaTime deltaTime = new DeltaTime();
    private final TrapezoidProfile.State profileGoal = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State profileSetpoint = new TrapezoidProfile.State(0, 0);

    private InternalGoal desiredGoal = InternalGoal.STOW;
    private InternalGoal currentGoal = InternalGoal.NONE;
    private double positionSetpointRots;

    private HoldMode holdMode = HoldMode.STIFF;
    private final LoggedTrigger squishyModeTrigger = group.t("SquishyMode", this::atSetpoint).debounce(0.1);

    public IntakeSlide(final Constants.RobotMode mode, final HardwareConstants.IntakeSlideConstants constants) {
        this.intakeSlideIO = switch (mode) {
            case REAL -> new IntakeSlideIOReal(constants);
            case SIM -> new IntakeSlideIOSim(constants);
            case REPLAY, DISABLED -> new IntakeSlideIO() {};
        };

        this.inputs = new IntakeSlideIOInputsAutoLogged();

        squishyModeTrigger.onTrue(Commands.runOnce(() -> {
            holdMode = HoldMode.SQUISHY;
            setPositionImpl(positionSetpointRots);
        }));
    }

    @Override
    public void periodic() {
        final double intakePeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeSlideIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (MathUtil.isNear(positionSetpointRots, inputs.masterPositionRots, PositionToleranceRots)
                && MathUtil.isNear(0, inputs.masterVelocityRotsPerSec, VelocityToleranceRotsPerSec)
        ) {
            currentGoal = desiredGoal;
        } else {
            currentGoal = InternalGoal.NONE;
        }

        final double dt = deltaTime.get();
        final Goal goal = desiredGoal.goal;
        if (goal != null) {
            final GoalBehavior behavior = goal.behavior;
            if (behavior.type == ProfileType.WITH_VELOCITY) {
                profileSetpoint = behavior.profile.calculate(dt, profileSetpoint, profileGoal);
                setPositionVelocityImpl(profileSetpoint.position, profileSetpoint.velocity);
            }
        }

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal);
        Logger.recordOutput(LogKey + "/PositionSetpointRots", positionSetpointRots);
        Logger.recordOutput(LogKey + "/HoldMode", holdMode);
        Logger.recordOutput(LogKey + "/AtSetpoint", atSetpoint());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - intakePeriodicUpdateStart)
        );
    }

    public boolean atSetpoint() {
        return desiredGoal == currentGoal;
    }

    public Pose3d[] getComponentPoses() {
        final Pose3d slideExtended = SimConstants.IntakeSlide.ExtendedPose;
        final Pose3d slideRetracted = SimConstants.IntakeSlide.RetractedPose;

        final Pose3d hopperExtended = SimConstants.HopperExtension.ExtendedPose;
        final Pose3d hopperRetracted = SimConstants.HopperExtension.RetractedPose;

        final double extensionMeters = inputs.slideAveragePositionRots
                * SimConstants.IntakeSlide.SlideRotationsToLinearDistanceMetersRatio;
        final double totalExtensionDistance = slideExtended.getTranslation()
                .getDistance(slideRetracted.getTranslation());
        final double extensionRatio = extensionMeters / totalExtensionDistance;

        return new Pose3d[] {
                slideRetracted.interpolate(slideExtended, extensionRatio),
                hopperRetracted.interpolate(hopperExtended, extensionRatio)
        };
    }

    private void setPositionImpl(final double positionRots) {
        positionSetpointRots = positionRots;
        switch (holdMode) {
            case STIFF -> intakeSlideIO.toSlidePosition(positionRots);
            case SQUISHY -> intakeSlideIO.holdSlidePosition(positionRots);
        }
    }

    private void setPositionVelocityImpl(final double positionRots, final double velocityRotsPerSec) {
        positionSetpointRots = positionRots;
        intakeSlideIO.toSlidePositionVelocity(positionRots, velocityRotsPerSec);
    }

    private void setGoalImpl(final Goal goal) {
        if (desiredGoal.goal != goal || !atSetpoint()) {
            holdMode = HoldMode.STIFF;
        }

        desiredGoal = InternalGoal.fromGoal(goal);
        switch (goal.behavior.type) {
            case EXPO -> setPositionImpl(goal.positionRots);
            case WITH_VELOCITY -> {
                profileSetpoint.position = inputs.slideAveragePositionRots;
                profileSetpoint.velocity = inputs.slideAverageVelocityRotsPerSec;

                profileGoal.position = goal.positionRots;
                profileGoal.velocity = 0;
            }
        }
    }

    public Command toInstantGoal(final Goal goal) {
        return runOnce(() -> setGoalImpl(goal));
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setGoalImpl(goal),
                () -> setGoalImpl(Goal.STOW)
        );
    }

    public Command toGoalHold(final Goal goal) {
        return startEnd(
                () -> setGoalImpl(goal),
                () -> {
                    desiredGoal = InternalGoal.HOLD_POSITION;
                    setPositionImpl(inputs.slideAveragePositionRots);
                }
        );
    }
}
