package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.params.ShotParameters;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.turret.Turret;
import frc.robot.utils.Container;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.function.Function;
import java.util.function.Supplier;

public class Superstructure extends VirtualSubsystem {
    protected static final String LogKey = "Superstructure";

    public enum Goal {
        STOW(Turret.Goal.STOW, Hood.Goal.STOW, Shooter.Goal.IDLE),
        NO_VISION(Turret.Goal.NO_VISION, Hood.Goal.NO_VISION, Shooter.Goal.NO_VISION);

        private final Turret.Goal turretGoal;
        private final Hood.Goal hoodGoal;
        private final Shooter.Goal shooterGoal;

        Goal(final Turret.Goal turretGoal, final Hood.Goal hoodGoal, final Shooter.Goal shooterGoal) {
            this.turretGoal = turretGoal;
            this.hoodGoal = hoodGoal;
            this.shooterGoal = shooterGoal;
        }
    }

    private enum InternalGoal {
        NONE,
        DEFAULT,
        STOW(Goal.STOW),
        NO_VISION(Goal.NO_VISION),
        TRACKING,
        TRACKING_HOOD_DOWN;

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

    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;

    private InternalGoal desiredGoal = InternalGoal.STOW;

    public final Trigger atSetpoint;
    public final Trigger turretAtSetpoint;
    public final Trigger hoodAtSetpoint;
    public final Trigger shooterAtSetpoint;

    public final Trigger safeForTrench;

    public Superstructure(
            final Turret turret,
            final Hood hood,
            final Shooter shooter
    ) {
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;

        this.atSetpoint = turret.atSetpoint
                .and(hood.atSetpoint)
                .and(shooter.atSetpoint);
        this.turretAtSetpoint = turret.atSetpoint;
        this.hoodAtSetpoint = hood.atSetpoint;
        this.shooterAtSetpoint = shooter.atSetpoint;

        this.safeForTrench = hood.safeForTrench;
    }

    @Override
    public void periodic() {
        final double superstructurePeriodicUpdateStart = Timer.getFPGATimestamp();

        final InternalGoal currentGoal;
        if (atSetpoint.getAsBoolean()) {
            currentGoal = desiredGoal;
        } else {
            currentGoal = InternalGoal.NONE;
        }

        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal);
        Logger.recordOutput(LogKey + "/AtSetpoint", atSetpoint);
        Logger.recordOutput(LogKey + "/SafeForTrench", safeForTrench);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - superstructurePeriodicUpdateStart)
        );
    }

    public Command toGoal(final Goal goal) {
        return Commands.sequence(
                Commands.runOnce(() -> setDesiredGoal(goal)),
                Commands.parallel(
                        turret.toGoal(goal.turretGoal),
                        hood.toGoal(goal.hoodGoal),
                        shooter.toGoal(goal.shooterGoal)
                )
        )
                .finallyDo(() -> setDesiredGoal(Goal.STOW))
                .withName("ToGoal:" + goal);
    }

    @SuppressWarnings("unused")
    public Command setGoal(final Goal goal) {
        return Commands.runOnce(() -> setDesiredGoal(goal))
                .withName("SetGoal: " + goal);
    }

    public Command runParameters(final Supplier<ShotParameters> shotParametersSupplier) {
        return runParametersWithHood(
                shotParametersSupplier,
                cached -> hood.runPosition(
                        () -> cached.get().shooter().hoodPositionRots()
                )
        ).withName("RunParametersWithHoodStowed");
    }

    public Command runParametersWithHoodStowed(final Supplier<ShotParameters> shotParametersSupplier) {
        return runParametersWithHood(
                shotParametersSupplier,
                cached -> hood.runGoal(Hood.Goal.STOW)
        ).withName("RunParametersWithHoodStowed");
    }

    public boolean atSetpoint() {
        return atSetpoint.getAsBoolean();
    }

    public double getShooterVelocityRotsPerSec() {
        return shooter.getVelocityRotsPerSec();
    }

    public Transform2d getRobotToTurret() {
        return turret.getOffsetFromCenter();
    }

    public Translation2d getTurretTranslation(final Pose2d robotPose) {
        return robotPose.plus(getRobotToTurret()).getTranslation();
    }

    private Command runParametersWithHood(
            final Supplier<ShotParameters> shotParametersSupplier,
            final Function<Supplier<ShotParameters>, Command> hoodCommand
    ) {
        final Container<ShotParameters> params = Container.empty();
        final Supplier<ShotParameters> cached = () -> {
            if (params.hasValue()) {
                return params.get();
            }

            final ShotParameters newParams = shotParametersSupplier.get();
            params.set(newParams);
            return newParams;
        };

        return Commands.parallel(
                setInternalGoal(InternalGoal.TRACKING),
                turret.runPositionWithVelocity(
                        () -> cached.get().turretAngle().getRotations(),
                        () -> cached.get().turretVelocityRotsPerSec()
                ),
                hoodCommand.apply(cached),
                shooter.runVelocity(() -> cached.get().shooter().shooterVelocityRotsPerSec()),
                Commands.run(params::clear)
        ).withName("RunParameters");
    }

    private Command setInternalGoal(@SuppressWarnings("SameParameterValue") final InternalGoal goal) {
        return Commands.runOnce(() -> desiredGoal = goal)
                .withName("SetInternalGoal");
    }

    private void setDesiredGoal(final Goal goal) {
        desiredGoal = InternalGoal.fromGoal(goal);
    }
}