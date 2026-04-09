package frc.robot.subsystems.intake.slide;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.ext.SubsystemExt;
import frc.robot.utils.commands.trigger.LoggedTrigger;
import frc.robot.utils.control.DeltaTime;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

public class IntakeSlide extends SubsystemExt {
    protected static final String LogKey = "IntakeSlide";
    private static final double PositionToleranceRots = 0.4;
    private static final double VelocityToleranceRotsPerSec = 0.05;

    public enum Goal {
        STOW(0, GoalBehavior.expo()),
        SHOOTING(0, GoalBehavior.withVelocity(0.5, 0.5)),
        EXTEND(3.405, GoalBehavior.expo());

        public final double positionRots;
        private final GoalBehavior behavior;

        Goal(final double positionRots, final GoalBehavior behavior) {
            this.positionRots = positionRots;
            this.behavior = behavior;
        }
    }

    private enum InternalGoal {
        NONE,
        DEFAULT,
        STOW(Goal.STOW),
        EXTEND(Goal.EXTEND),
        SHOOTING(Goal.SHOOTING),
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
            final InternalGoal internalGoal = GoalToInternal.get(goal);

            return internalGoal == null ? DEFAULT : internalGoal;
        }

        public final Goal goal;

        InternalGoal(final Goal goal) {
            this.goal = goal;
        }

        InternalGoal() {
            this(null);
        }
    }

    private enum ProfileType {
        EXPO,
        WITH_VELOCITY
    }

    private static class GoalBehavior {
        public final ProfileType type;
        public final double maxVelocity;
        public final double maxAcceleration;

        public final TrapezoidProfile profile;

        public GoalBehavior(final ProfileType type, final double maxVelocity, final double maxAcceleration) {
            this.type = type;
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;

            this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        }

        public static GoalBehavior expo() {
            return new GoalBehavior(ProfileType.EXPO, 0, 0);
        }

        public static GoalBehavior withVelocity(final double maxVelocity, final double maxAcceleration) {
            return new GoalBehavior(ProfileType.WITH_VELOCITY, maxVelocity, maxAcceleration);
        }
    }

    private enum HoldMode {
        SOFT,
        HARD
    }

    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final HardwareConstants.IntakeSlideConstants constants;

    private final IntakeSlideIO intakeSlideIO;
    private final IntakeSlideIOInputsAutoLogged inputs = new IntakeSlideIOInputsAutoLogged();

    private final DeltaTime deltaTime = new DeltaTime();
    private final TrapezoidProfile.State profileGoal = new TrapezoidProfile.State(0,0);
    private TrapezoidProfile.State profileSetpoint = new TrapezoidProfile.State(0,0);

    private InternalGoal desiredGoal = InternalGoal.STOW;
    private double positionSetpointRots = 0.0;

    private HoldMode holdMode = HoldMode.HARD;

    public final LoggedTrigger atSetpoint;
    public final LoggedTrigger isIntakeStopped;

    public IntakeSlide(final Constants.RobotMode mode, final HardwareConstants.IntakeSlideConstants constants) {
        this.constants = constants;

        this.intakeSlideIO = switch (mode) {
            case REAL -> new IntakeSlideIOReal(constants);
            case SIM -> new IntakeSlideIOSim(constants);
            case REPLAY, DISABLED -> new IntakeSlideIO() {};
        };

        this.atSetpoint = group.t("AtSetpoint", () -> atPosition(positionSetpointRots));
        this.isIntakeStopped = group.t("IsIntakeStopped", () -> MathUtil.isNear(
                0,
                getVelocityRotsPerSec(),
                0.1
        )).debounce(0.2);

        this.intakeSlideIO.zero();

        final LoggedTrigger softModeTrigger = group.t("SoftMode", this::atSetpoint).debounce(0.1);
        softModeTrigger.onTrue(Commands.runOnce(() -> {
            holdMode = HoldMode.SOFT;
            setDesiredPosition(positionSetpointRots);
        }));
    }

    @Override
    public void periodic() {
        final double intakeSlidePeriodicUpdateStart = Timer.getFPGATimestamp();

        intakeSlideIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        final InternalGoal currentGoal;
        if (atSetpoint()) {
            currentGoal = desiredGoal;
        } else {
            currentGoal = InternalGoal.NONE;
        }

        final double dt = deltaTime.get();
        final Goal goal = desiredGoal.goal;
        if (goal != null) {
            final GoalBehavior goalBehavior = goal.behavior;
            if (goalBehavior.type == ProfileType.WITH_VELOCITY) {
                profileSetpoint = goalBehavior.profile.calculate(dt, profileSetpoint, profileGoal);
                setDesiredPositionWithVelocity(profileSetpoint.position, profileSetpoint.velocity);
            }
        }

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal);
        Logger.recordOutput(LogKey + "/AtSetpoint", atSetpoint);
        Logger.recordOutput(LogKey + "/PositionSetpointRots", positionSetpointRots);
        Logger.recordOutput(LogKey + "/HoldMode", holdMode);

        Logger.recordOutput(LogKey + "/AtUpperLimit", atUpperLimit());
        Logger.recordOutput(LogKey + "/AtLowerLimit", atLowerLimit());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - intakeSlidePeriodicUpdateStart)
        );
    }

    public Command toGoal(final Goal goal) {
        return startEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.STOW)
        ).withName("ToGoal: " + goal);
    }

    public Command toGoalHold(final Goal goal) {
        return startEnd(
                () -> setDesiredGoal(goal),
                () -> {
                    desiredGoal = InternalGoal.HOLD_POSITION;
                    setDesiredPosition(inputs.averagePositionRots);
                }
        ).withName("ToGoalHold: " + goal);
    }

    public Command setGoal(final Goal goal) {
        return runOnce(() -> setDesiredGoal(goal))
                .withName("SetGoal: " + goal);
    }

    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(inputs.averagePositionRots);
    }

    public double getVelocityRotsPerSec() {
        return inputs.averageVelocityRotsPerSec;
    }

    public LoggedTrigger atGoal(final Goal goal) {
        return group.t("AtGoal: " + goal, () -> desiredGoal == InternalGoal.fromGoal(goal)
                && atPosition(goal.positionRots));
    }

    private void setDesiredGoal(final Goal goal) {
        if (desiredGoal.goal != goal || !atSetpoint.getAsBoolean()) {
            holdMode = HoldMode.HARD;
        }

        desiredGoal = InternalGoal.fromGoal(goal);
        switch (goal.behavior.type) {
            case EXPO -> setDesiredPosition(goal.positionRots);
            case WITH_VELOCITY -> {
                profileSetpoint.position = inputs.averagePositionRots;
                profileSetpoint.velocity = inputs.averageVelocityRotsPerSec;

                profileGoal.position = goal.positionRots;
                profileGoal.velocity = 0;
            }
        }
    }

    private void setDesiredPosition(final double positionRots) {
        positionSetpointRots = positionRots;
        switch (holdMode) {
            case SOFT -> intakeSlideIO.holdSlidePosition(positionRots);
            case HARD -> intakeSlideIO.toSlidePosition(positionRots);
        }
    }

    private void setDesiredPositionWithVelocity(final double positionRots, final double velocityRotsPerSec) {
        positionSetpointRots = positionRots;
        intakeSlideIO.toSlidePositionWithVelocity(positionRots, velocityRotsPerSec);
    }

    private boolean atPosition(final double positionSetpoint) {
        return MathUtil.isNear(
                positionSetpoint,
                inputs.averagePositionRots,
                PositionToleranceRots
        ) && MathUtil.isNear(
                0,
                inputs.averageVelocityRotsPerSec,
                VelocityToleranceRotsPerSec
        );
    }

    private boolean atSetpoint() {
        return atSetpoint.getAsBoolean();
    }

    private boolean atUpperLimit() {
        return inputs.masterPositionRots >= constants.forwardLimitRots();
    }

    private boolean atLowerLimit() {
        return inputs.masterPositionRots <= constants.reverseLimitRots();
    }
}
