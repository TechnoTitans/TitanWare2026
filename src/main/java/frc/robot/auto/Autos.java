package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Robot;
import frc.robot.ShootCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.ShotCalculator;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.utils.commands.LoggedTrigger;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Autos {
    public static final String LogKey = "Auto";
    private static final int SHOOTING_TIME = 6;

    private final Swerve swerve;
    private final Intake intake;
    private final Indexer indexer;
    private final Superstructure superstructure;

    private final AutoFactory autoFactory;

    private final Supplier<ShotCalculator.ShotCalculation> staticShotCalculation;
    private final Supplier<ShotCalculator.ShotCalculation> movingShotCalculation;

    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final LoggedTrigger robotStopped;
    private final LoggedTrigger targetIsHub;
    private final LoggedTrigger turretSafe;

    public Autos(
            final Swerve swerve,
            final Intake intake,
            final Indexer indexer,
            final Superstructure superstructure,
            final PhotonVision photonVision
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.indexer = indexer;
        this.superstructure = superstructure;

        this.autoFactory = new AutoFactory(
                swerve::getPose,
                photonVision::resetPose,
                swerve::followChoreoSample,
                true,
                swerve,
                (trajectory, trajectoryRunning) -> {
                    Logger.recordOutput(
                            Autos.LogKey + "/Trajectory/Path",
                            (Robot.IsRedAlliance.getAsBoolean() ? trajectory.flipped() : trajectory).getPoses()
                    );

                    Logger.recordOutput(
                            Autos.LogKey + "/Trajectory/Name",
                            trajectory.name()
                    );

                    Logger.recordOutput(
                            Autos.LogKey + "/Trajectory/Running",
                            trajectoryRunning
                    );
                }
        );

        this.staticShotCalculation = staticParameters(swerve::getPose);
        this.movingShotCalculation = ShotCalculator.getMovingShotCalculationSupplier(
                swerve::getPose,
                swerve::getRobotRelativeSpeeds,
                FieldConstants::getHubPose
        );

        this.robotStopped = group.t("RobotStopped",
                () -> ShootCommands.linearSpeed(swerve.getFieldRelativeSpeeds()) <= 0.01
        );
        this.targetIsHub = group.t("TargetIsHub",
                () -> ShootCommands.getTarget(swerve.getPose()) == ShootCommands.Target.HUB);
        this.turretSafe = group.t("TurretSafe",
                () -> {
                    final double safeXClose = FieldConstants.getTurretSafeXCloseBoundary();
                    final double safeXFar = FieldConstants.getTurretSafeXFarBoundary();
                    final double turretX = superstructure
                            .getTurretTranslation(swerve.getPose())
                            .getX();
                    return Robot.IsRedAlliance.getAsBoolean()
                            ? (turretX >= safeXClose || turretX <= safeXFar)
                            : (turretX <= safeXClose || turretX >= safeXFar);
                }
        );
    }

    private Command runStartingTrajectory(final AutoTrajectory startingTrajectory) {
        return Commands.sequence(
                startingTrajectory.resetOdometry(),
                startingTrajectory.cmd()
        ).withName("RunStartingTrajectory");
    }

    private Supplier<ShotCalculator.ShotCalculation> staticParameters(final Supplier<Pose2d> robotPoseSupplier) {
        return ShotCalculator.getStaticShotCalculationSupplier(
                robotPoseSupplier,
                swerve::getFieldRelativeSpeeds,
                FieldConstants::getHubPose
        );
    }

    private Supplier<ShotCalculator.ShotCalculation> staticParametersFromPose(final Pose2d pose) {
        return staticParameters(() -> pose);
    }

    private Supplier<ShotCalculator.ShotCalculation> staticParametersFromFinalPose(final AutoTrajectory trajectory) {
        return trajectory.getFinalPose()
                .map(this::staticParametersFromPose)
                .orElse(staticShotCalculation);
    }

    private Command intakeFromTrench(
            final Supplier<ShotCalculator.ShotCalculation> fixed,
            final Supplier<ShotCalculator.ShotCalculation> tracking
    ) {
        return parallel(
                intake.intake(),
                sequence(
                        waitUntil(targetIsHub.negate()
                                .and(turretSafe)),
                        superstructure.runParametersWithHoodStowed(fixed)
                                .until(targetIsHub.and(turretSafe)),
                        superstructure.runParameters(tracking)
                                .onlyIf(turretSafe)
                )
        ).withName("IntakeFromTrench");
    }

    private Command shootStatic() {
        final Timer timer = new Timer();

        return deadline(
                repeatingSequence(
                        waitUntil(robotStopped
                                .and(superstructure.atSetpoint)),
                        runOnce(timer::start),
                        deadline(
                                indexer.feed()
                                        .onlyWhile(robotStopped
                                                .and(superstructure.atSetpoint))
                                        .finallyDo(timer::stop),
                                intake.stowFeed()
                        )
                ).until(() -> timer.hasElapsed(SHOOTING_TIME)),
                superstructure.runParameters(staticShotCalculation)
                                .onlyIf(turretSafe),
                swerve.runWheelXCommand(),
                Commands.run(
                        () -> Logger.recordOutput("RobotStopped", robotStopped)
                ),
                runOnce(timer::reset)
        )
                .withName("ShootStatic");
    }

    private Command shootWhileMoving() {
        final Timer timer = new Timer();

        return deadline(
                repeatingSequence(
                        waitUntil(superstructure.atSetpoint),
                        runOnce(timer::start),
                        deadline(
                                indexer.feed()
                                        .onlyWhile(superstructure.atSetpoint)
                                        .finallyDo(timer::stop),
                                intake.stowFeed()
                        )
                ).until(() -> timer.hasElapsed(SHOOTING_TIME)),
                superstructure.runParameters(movingShotCalculation)
                        .onlyIf(turretSafe)
        ).withName("ShootWhileMoving");
    }

    public AutoRoutine doNothing() {
        final AutoRoutine routine = autoFactory.newRoutine("DoNothing");

        routine.active().whileTrue(
                Commands.waitUntil(RobotModeTriggers.autonomous().negate())
        );

        return routine;
    }

    public AutoRoutine onlyShootPreload() {
        final AutoRoutine routine = autoFactory.newRoutine("OnlyShootPreload");

        routine.active().onTrue(
                Commands.sequence(
                        shootStatic()
                )
        );

        return routine;
    }

    public AutoRoutine leftCenterLineDepot() {
        final AutoRoutine routine = autoFactory.newRoutine("LeftCenterLineDepot");
        final AutoTrajectory startToCenterLineAndBack = routine.trajectory("LeftStartToCenterLineAndBack");
        final AutoTrajectory shootingToDepot = routine.trajectory("LeftShootingToDepot");

        routine.active().onTrue(
                runStartingTrajectory(startToCenterLineAndBack)
        );

        startToCenterLineAndBack.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(startToCenterLineAndBack),
                        staticShotCalculation
                )
        );

        startToCenterLineAndBack.done().onTrue(
                sequence(
                        shootStatic(),
                        deadline(
                                waitUntil(superstructure.safeForTrench)
                                        .andThen(shootingToDepot.cmd())
                                        .asProxy(),
                                superstructure.runParametersWithHoodStowed(staticShotCalculation).asProxy()
                        )
                )
        );

        shootingToDepot.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(shootingToDepot),
                        staticShotCalculation
                )
        );

        shootingToDepot.done().onTrue(shootStatic());

        return routine;
    }

    public AutoRoutine rightCenterLineOutpost() {
        final AutoRoutine routine = autoFactory.newRoutine("RightCenterLineOutpost");
        final AutoTrajectory startToCenterLineAndBack = routine.trajectory("RightStartToCenterLineAndBack");
        final AutoTrajectory shootingToOutpost = routine.trajectory("RightShootingToOutput");

        routine.active().onTrue(
                runStartingTrajectory(startToCenterLineAndBack)
        );

        startToCenterLineAndBack.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(startToCenterLineAndBack),
                        staticShotCalculation
                )
        );

        startToCenterLineAndBack.done().onTrue(
                sequence(
                        shootStatic(),
                        deadline(
                                waitUntil(superstructure.safeForTrench)
                                        .andThen(shootingToOutpost.cmd())
                                        .asProxy(),
                                superstructure.runParametersWithHoodStowed(staticShotCalculation).asProxy()
                        )
                )
        );

        shootingToOutpost.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(shootingToOutpost),
                        staticShotCalculation
                )
        );

        shootingToOutpost.done().onTrue(shootStatic());

        return routine;
    }

    public AutoRoutine rightDoubleSweep() {
        final AutoRoutine routine = autoFactory.newRoutine("RightDoubleSweep");
        final AutoTrajectory startToCenterLineAndBack = routine.trajectory("RightStartToCenterLineAndBack");
        final AutoTrajectory rightSweep = routine.trajectory("RightSweep");

        routine.active().onTrue(
                runStartingTrajectory(startToCenterLineAndBack)
        );

        startToCenterLineAndBack.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(startToCenterLineAndBack),
                        staticShotCalculation
                )
        );

        startToCenterLineAndBack.done().onTrue(
                sequence(
                        shootStatic(),
                        deadline(
                                waitUntil(superstructure.safeForTrench)
                                        .andThen(rightSweep.cmd())
                                        .asProxy(),
                                superstructure.runParametersWithHoodStowed(staticShotCalculation).asProxy()
                        )
                )
        );

        rightSweep.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(rightSweep),
                        staticShotCalculation
                )
        );

        rightSweep.done().onTrue(shootStatic());

        return routine;
    }
}