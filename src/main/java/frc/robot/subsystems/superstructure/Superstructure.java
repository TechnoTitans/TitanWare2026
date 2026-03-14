package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.turret.Turret;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Superstructure extends VirtualSubsystem {
    protected static final String LogKey = "Superstructure";

    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;

    private Goal desiredGoal = Goal.TRACKING;
    private Goal runningGoal = desiredGoal;

    private final EventLoop eventLoop;

    private final Trigger desiredGoalIsRunningGoal;
    private final Trigger desiredGoalChanged;

    public final Trigger atSetpoint;
    public final Trigger atHoodSetpoint;
    public final Trigger atShooterSetpoint;
    public final Trigger atTurretSetpoint;

    private final Supplier<ShotCalculator.ShotCalculation> shotCalculationSupplier;

    public enum Goal {
        TRACKING(Turret.Goal.TRACKING, Hood.Goal.STOW, Shooter.Goal.TRACKING, true),
        STATIC_SHOT_PREP(Turret.Goal.TRACKING, Hood.Goal.STOW, Shooter.Goal.TRACKING, false),
        SHOOTING(Turret.Goal.TRACKING, Hood.Goal.SHOOTING, Shooter.Goal.TRACKING, true);

        private final Turret.Goal turretGoal;
        private final Hood.Goal hoodGoal;
        private final Shooter.Goal shooterGoal;

        private final boolean isDynamic;

        Goal(
                final Turret.Goal turretGoal,
                final Hood.Goal hoodGoal,
                final Shooter.Goal shooterGoal,
                final boolean isDynamic
        ) {
            this.turretGoal = turretGoal;
            this.hoodGoal = hoodGoal;
            this.shooterGoal = shooterGoal;
            this.isDynamic = isDynamic;
        }
    }

    public Superstructure(
            final Turret turret,
            final Hood hood,
            final Shooter shooter,
            final Supplier<ShotCalculator.ShotCalculation> shotCalculationSupplier
    ) {
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;

        this.eventLoop = new EventLoop();

        this.desiredGoalIsRunningGoal = new Trigger(eventLoop, () -> desiredGoal == runningGoal);
        this.desiredGoalChanged = new Trigger(eventLoop, () -> desiredGoal != runningGoal);

        this.atTurretSetpoint = turret.atSetpoint;
        this.atShooterSetpoint = shooter.atSetpoint;
        this.atHoodSetpoint = hood.atSetpoint;

        this.atSetpoint = turret.atSetpoint
                .and(hood.atSetpoint)
                .and(shooter.atSetpoint)
                .debounce(0.1, Debouncer.DebounceType.kFalling);

        this.shotCalculationSupplier = shotCalculationSupplier;

        turret.setGoal(desiredGoal.turretGoal);
        hood.setGoal(desiredGoal.hoodGoal);
        shooter.setGoal(desiredGoal.shooterGoal);
    }

    private Command runEnd(final Runnable run, final Runnable end) {
        return Commands.runEnd(run, end, turret, hood);
    }

    private Command run(final Runnable run) {
        return Commands.run(run, turret, hood);
    }

    @Override
    public void periodic() {
        eventLoop.poll();

        if (desiredGoal.isDynamic) {
            final ShotCalculator.ShotCalculation shotCalculation = shotCalculationSupplier.get();

            turret.updatePositionSetpoint(shotCalculation.desiredTurretRotation().getRotations());
            hood.updateShootingDesiredPosition(shotCalculation.desiredHoodRotationRots());
            shooter.updateVelocitySetpoint(shotCalculation.desiredShooterVelocity());
        }

        if (desiredGoal != runningGoal) {
            turret.setGoal(desiredGoal.turretGoal);
            hood.setGoal(desiredGoal.hoodGoal);
            shooter.setGoal(desiredGoal.shooterGoal);

            runningGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/RunningGoal", runningGoal);
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal);

        Logger.recordOutput(LogKey + "/AtSetpoint", atSetpoint);

        Logger.recordOutput(LogKey + "/Triggers/DesiredGoalIsRunningGoal", desiredGoalIsRunningGoal);
        Logger.recordOutput(LogKey + "/Triggers/DesiredGoalChanged", desiredGoalChanged);
    }

    public Transform2d getOffsetFromCenter() {
        return turret.getOffsetFromCenter();
    }

    public Translation2d getTurretTranslation(final Pose2d robotPose) {
        return robotPose.plus(getOffsetFromCenter()).getTranslation();
    }

    public double getShooterVelocityRotsPerSec() {
        return shooter.getVelocityRotsPerSec();
    }

    public void updateStaticShotParameter(
            final ShotCalculator.ShotCalculation shotCalculation
    ) {
        turret.updatePositionSetpoint(shotCalculation.desiredTurretRotation().getRotations());
        hood.updateShootingDesiredPosition(shotCalculation.desiredHoodRotationRots());
        shooter.updateVelocitySetpoint(shotCalculation.desiredShooterVelocity());
    }


    public Trigger atSetpoint(final Goal goal) {
        return atSetpoint(() -> goal);
    }

    public Command setGoalCommand(final Goal goal) {
        return Commands.runOnce(() -> setDesiredGoal(goal)).withName("SetGoal: " + goal.toString());
    }

    public Command toGoal(final Supplier<Goal> goal) {
        return Commands.startEnd(
                () -> setDesiredGoal(goal.get()),
                () -> setDesiredGoal(Goal.TRACKING)
        ).withName("ToGoalSupplier");
    }

    public Command toGoal(final Goal goal) {
        return Commands.startEnd(
                () -> setDesiredGoal(goal),
                () -> setDesiredGoal(Goal.TRACKING)
        ).withName("ToGoal: " + goal.toString());
    }

    public Command runGoal(final Supplier<Goal> goalSupplier) {
        return run(() -> setDesiredGoal(goalSupplier.get())).withName("RunGoalSupplier");
    }

    public Trigger atSetpoint(final Supplier<Goal> goalSupplier) {
        return atSetpoint.and(() -> runningGoal == goalSupplier.get());
    }

    private void setDesiredGoal(final Goal desiredGoal) {
        this.desiredGoal = desiredGoal;
        eventLoop.poll();
    }
}