package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.turret.Turret;
import frc.robot.utils.Container;
import frc.robot.utils.commands.trigger.LoggedTrigger;
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
        TRACKING;

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
    private InternalGoal currentGoal = InternalGoal.NONE;

    public final LoggedTrigger safeForTrench;

    public Superstructure(
            final Turret turret,
            final Hood hood,
            final Shooter shooter
    ) {
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;

        this.safeForTrench = hood.safeForTrench;
    }

    @Override
    public void periodic() {
        final double superstructurePeriodicUpdateStart = Timer.getFPGATimestamp();

        if (hood.atSetpoint() && shooter.atSetpoint() && turret.atSetpoint()) {
            currentGoal = desiredGoal;
        } else {
            currentGoal = InternalGoal.NONE;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal);
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - superstructurePeriodicUpdateStart)
        );
    }

    //TODO: Make toGoalLike
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

    public Command runParameters(final Supplier<ShotCalculator.ShotCalculation> shotCalculationSupplier) {
        return runParametersWithHood(
                shotCalculationSupplier,
                cached -> hood.runPosition(() -> cached.get().hoodRotationRots())
        ).withName("RunParametersWithHoodStowed");
    }

    public Command runParametersWithHoodStowed(final Supplier<ShotCalculator.ShotCalculation> shotCalculationSupplier) {
        return runParametersWithHood(
                shotCalculationSupplier,
                cached -> hood.runGoal(Hood.Goal.STOW)
        ).withName("RunParametersWithHoodStowed");
    }

    public boolean atSetpoint() {
        return currentGoal == desiredGoal;
    }

    public double getShooterVelocityRotsPerSec() {
        return shooter.getVelocityRotsPerSec();
    }

    private Command runParametersWithHood(
            final Supplier<ShotCalculator.ShotCalculation> shotCalculationSupplier,
            final Function<Supplier<ShotCalculator.ShotCalculation>, Command> hoodCommand
    ) {
        final Container<ShotCalculator.ShotCalculation> calculation = Container.empty();
        final Supplier<ShotCalculator.ShotCalculation> cached = () -> {
            if (calculation.hasValue()) {
                return calculation.get();
            }

            final ShotCalculator.ShotCalculation newCalculation = shotCalculationSupplier.get();
            calculation.set(newCalculation);
            return newCalculation;
        };

        return Commands.parallel(
                setInternalGoal(InternalGoal.TRACKING),
                turret.runPositionWithVelocity(
                        () -> cached.get().turretRotationRots(),
                        () -> cached.get().turretSpeedRotsPerSec()
                ),
                hoodCommand.apply(cached),
                shooter.runVelocity(() -> cached.get().shooterVelocityRotsPerSec()),
                Commands.run(calculation::clear)
        ).withName("RunParameters");
    }

    private Command setInternalGoal(final InternalGoal goal) {
        return Commands.runOnce(() -> desiredGoal = goal)
                .withName("SetInternalGoal");
    }

    private void setDesiredGoal(final Goal goal) {
        desiredGoal = InternalGoal.fromGoal(goal);
    }
}