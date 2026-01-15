package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.hopper.Hopper;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.turret.Turret;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Superstructure extends VirtualSubsystem {
    protected static final String LogKey = "Superstructure";

    private final Hopper hopper;
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;

    private boolean shouldShoot = false;

    private Goal desiredGoal = Goal.TRACKING;
    private Goal runningGoal = desiredGoal;
    private Goal atGoal = desiredGoal;

    private final EventLoop eventLoop;

    private final Trigger desiredGoalIsRunningGoal;
    private final Trigger desiredGoalIsAtGoal;
    private final Trigger desiredGoalChanged;

    public final Trigger atSuperstructureSetpoint;

    private final Supplier<ShotCalculator.ShotCalculation> shotCalculationSupplier;

    public enum Goal {
        STOW(Hopper.Goal.STOP, Turret.Goal.STOW, Hood.Goal.STOW, Shooter.Goal.STOP, false),
        CLIMB(Hopper.Goal.STOP, Turret.Goal.CLIMB, Hood.Goal.CLIMB, Shooter.Goal.STOP, false),
        TRACKING(Hopper.Goal.FEED, Turret.Goal.TRACKING_HUB, Hood.Goal.TRACKING_HUB, Shooter.Goal.TRACKING_HUB, true),
        FERRYING(Hopper.Goal.FEED, Turret.Goal.FERRYING, Hood.Goal.FERRYING, Shooter.Goal.FERRYING, true),
        EJECTING(Hopper.Goal.EJECT, Turret.Goal.STOW, Hood.Goal.STOW, Shooter.Goal.STOP, false);

        private final Hopper.Goal hopperGoal;
        private final Turret.Goal turretGoal;
        private final Hood.Goal hoodGoal;
        private final Shooter.Goal shooterGoal;

        private final boolean isDynamic;

        Goal(final Hopper.Goal hopperGoal, final Turret.Goal turretGoal, final Hood.Goal hoodGoal, final Shooter.Goal shooterGoal, final boolean isDynamic) {
            this.hopperGoal = hopperGoal;
            this.turretGoal = turretGoal;
            this.hoodGoal = hoodGoal;
            this.shooterGoal = shooterGoal;
            this.isDynamic = isDynamic;
        }
    }

    public Superstructure(
            final Hopper hopper,
            final Turret turret,
            final Hood hood,
            final Shooter shooter,
            final Supplier<ShotCalculator.ShotCalculation> shotCalculationSupplier
    ) {
        this.hopper = hopper;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;

        this.eventLoop = new EventLoop();

        this.desiredGoalIsRunningGoal = new Trigger(eventLoop, () -> this.desiredGoal == runningGoal);
        this.desiredGoalChanged = new Trigger(eventLoop, () -> this.desiredGoal != runningGoal);
        this.desiredGoalIsAtGoal = new Trigger(eventLoop, () -> this.desiredGoal == atGoal);
        this.atSuperstructureSetpoint = turret.atSetpoint
                .and(hood.atSetpoint)
                .and(shooter.atVelocitySetpoint);

        this.shotCalculationSupplier = shotCalculationSupplier;

        turret.setGoal(desiredGoal.turretGoal);
        hood.setGoal(desiredGoal.hoodGoal);
    }

    private Command runOnce(final Runnable action) {
        return Commands.runOnce(action, turret, hood);
    }

    private Command runEnd(final Runnable run, final Runnable end) {
        return Commands.runEnd(run, end, turret, hood);
    }

    private Command run(final Runnable run) {
        return Commands.run(run, turret, hood);
    }

    private void setDesiredGoal(final Goal desiredGoal) {
        this.desiredGoal = desiredGoal;
        eventLoop.poll();
    }

    @Override
    public void periodic() {
        eventLoop.poll();

        if (desiredGoal.isDynamic) {
            final ShotCalculator.ShotCalculation shotCalculation = shotCalculationSupplier.get();

            turret.updatePositionSetpoint(shotCalculation.desiredTurretRotation().getRotations());
            hood.updateDesiredHoodPosition(shotCalculation.hoodShooterCalculation().hoodRotation().getRotations());

            if (shouldShoot) {
                shooter.updateVelocitySetpoint(shotCalculation.hoodShooterCalculation().flywheelVelocity());
            }
        }

        if (desiredGoal != runningGoal) {
            turret.setGoal(desiredGoal.turretGoal);
            hood.setGoal(desiredGoal.hoodGoal);
            shooter.setGoal(desiredGoal.shooterGoal);
            hopper.setGoal(desiredGoal.hopperGoal);

            this.runningGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/RunningGoal", runningGoal);
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);
        Logger.recordOutput(LogKey + "/AtGoal", atGoal);

        Logger.recordOutput(LogKey + "/AtSetpoint", atSuperstructureSetpoint);

        Logger.recordOutput(LogKey + "/Triggers/DesiredGoalIsRunningGoal", desiredGoalIsRunningGoal);
        Logger.recordOutput(LogKey + "/Triggers/DesiredGoalIsAtGoal", desiredGoalIsAtGoal);
        Logger.recordOutput(LogKey + "/Triggers/DesiredGoalChanged", desiredGoalChanged);
    }

    public Trigger atSetpoint(final Supplier<Goal> goalSupplier) {
        return atSuperstructureSetpoint.and(() -> atGoal == goalSupplier.get());
    }

    public Trigger atSetpoint(final Goal goal) {
        return atSetpoint(() -> goal);
    }

    public Command toGoal(final Supplier<Goal> goal) {
        return runEnd(
                () -> setDesiredGoal(goal.get()),
                () -> setDesiredGoal(Goal.TRACKING)
        ).withName("ToGoal");
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.TRACKING)
        ).withName("ToGoal");
    }

    public Command runGoal(final Supplier<Goal> goalSupplier) {
        return run(() -> setDesiredGoal(goalSupplier.get()))
                .withName("RunGoal");
    }

    public Command setShootingState(final boolean shouldShoot) {
        return Commands.runOnce(
                () -> this.shouldShoot = shouldShoot
        ).withName("SetShootingState");
    }
}
