package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.FuelState;
import frc.robot.Robot;
import frc.robot.ShootCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.params.MovingTOFShot;
import frc.robot.subsystems.superstructure.params.ShotParameters;
import frc.robot.subsystems.superstructure.params.ShotProvider;
import frc.robot.subsystems.superstructure.params.StaticShot;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.utils.commands.trigger.LoggedTrigger;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Autos {
    public static final String LogKey = "Auto";

    @SuppressWarnings("FieldCanBeLocal")
    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final Swerve swerve;
    private final Intake intake;
    private final Indexer indexer;
    private final FuelState fuelState;
    private final Superstructure superstructure;

    private final AutoFactory autoFactory;

    private final ShotProvider<ShotProvider.Kind.Static> staticShotProvider;
    private final Supplier<ShotParameters> staticShot;

    private final ShotProvider<ShotProvider.Kind.Moving> movingShotProvider;
    private final Supplier<ShotParameters> movingShot;

    private final LoggedTrigger robotStopped;
    private final LoggedTrigger targetIsHub;
    private final LoggedTrigger turretSafe;

    public Autos(
            final Swerve swerve,
            final Intake intake,
            final Indexer indexer,
            final Superstructure superstructure,
            final PhotonVision photonVision,
            final FuelState fuelState
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.indexer = indexer;
        this.superstructure = superstructure;
        this.fuelState = fuelState;

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

        this.staticShotProvider = new StaticShot();
        this.staticShot = staticParameters(swerve::getPose);

        this.movingShotProvider = new MovingTOFShot();
        this.movingShot = movingParameters(FieldConstants::getHubPose);

        this.robotStopped = group.t(
                "RobotStopped",
                () -> ShootCommands.linearSpeed(swerve.getFieldRelativeSpeeds()) <= 0.01
        );
        this.targetIsHub = group.t(
                "TargetIsHub",
                () -> ShootCommands.getTarget(superstructure.getTurretTranslation(swerve.getPose()))
                        == ShootCommands.Target.HUB
        );
        this.turretSafe = group.t(
                "TurretSafe",
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

    public AutoRoutine doNothing() {
        final AutoRoutine routine = autoFactory.newRoutine("DoNothing");

        routine.active().whileTrue(Commands.waitUntil(RobotModeTriggers.autonomous().negate()));

        return routine;
    }

    public AutoRoutine onlyShootPreload() {
        final AutoRoutine routine = autoFactory.newRoutine("OnlyShootPreload");

        routine.active().onTrue(shootStatic());

        return routine;
    }

    public AutoRoutine leftSweepDepot() {
        final AutoRoutine routine = autoFactory.newRoutine("LeftSweepDepot");
        final AutoTrajectory startToCenterLineAndBack = routine.trajectory("LeftStartToCenterLineAndBack");
        final AutoTrajectory shootingToDepot = routine.trajectory("LeftShootingToDepot");

        routine.active().onTrue(parallel(
                runStartingTrajectory(startToCenterLineAndBack),
                runOnce(fuelState::setSimFuelPreloaded)
        ));

        startToCenterLineAndBack.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(startToCenterLineAndBack),
                        staticShot
                )
        );

        startToCenterLineAndBack.done().onTrue(
                sequence(
                        shootStatic(),
                        deadline(
                                waitUntil(superstructure.safeForTrench)
                                        .andThen(shootingToDepot.cmd())
                                        .asProxy(),
                                superstructure.runParametersWithHoodStowed(staticShot).asProxy()
                        )
                )
        );

        shootingToDepot.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(shootingToDepot),
                        staticShot
                )
        );

        shootingToDepot.done().onTrue(shootStatic());

        return routine;
    }

    public AutoRoutine rightSweepOutpost() {
        final AutoRoutine routine = autoFactory.newRoutine("RightSweepOutpost");
        final AutoTrajectory firstSweep = routine.trajectory("RightFirstSweep");
        final AutoTrajectory shootingToOutpost = routine.trajectory("RightShootingToOutput");

        routine.active().onTrue(parallel(
                runStartingTrajectory(firstSweep),
                runOnce(fuelState::setSimFuelPreloaded)
        ));

        firstSweep.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(firstSweep),
                        staticShot
                )
        );

        firstSweep.done().onTrue(
                sequence(
                        shootStatic(),
                        deadline(
                                waitUntil(superstructure.safeForTrench)
                                        .andThen(shootingToOutpost.cmd())
                                        .asProxy(),
                                superstructure.runParametersWithHoodStowed(staticShot).asProxy()
                        )
                )
        );

        shootingToOutpost.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(shootingToOutpost),
                        staticShot
                )
        );

        shootingToOutpost.done().onTrue(shootStatic());

        return routine;
    }

    public AutoRoutine leftDoubleSweep() {
        final AutoRoutine routine = autoFactory.newRoutine("LeftDoubleSweep");
        final AutoTrajectory firstSweep = routine.trajectory("LeftFirstSweep");
        final AutoTrajectory secondSweep = routine.trajectory("LeftSweep");

        routine.active().onTrue(parallel(
                runStartingTrajectory(firstSweep),
                runOnce(fuelState::setSimFuelPreloaded)
        ));

        firstSweep.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(firstSweep),
                        staticShot
                )
        );

        firstSweep.done().onTrue(
                sequence(
                        shootStatic(),
                        deadline(
                                waitUntil(superstructure::atSetpoint)
                                    .andThen(secondSweep.cmd())
                                    .asProxy(),
                                superstructure.runParametersWithHoodStowed(staticShot).asProxy()
                        )
                )
        );

        secondSweep.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(secondSweep),
                        staticShot
                )
        );

        secondSweep.done().onTrue(shootStatic());

        return routine;
    }

    public AutoRoutine rightDoubleSweep() {
        final AutoRoutine routine = autoFactory.newRoutine("RightDoubleSweep");
        final AutoTrajectory firstSweep = routine.trajectory("RightFirstSweep");
        final AutoTrajectory secondSweep = routine.trajectory("RightSweep");

        routine.active().onTrue(parallel(
                runStartingTrajectory(firstSweep),
                runOnce(fuelState::setSimFuelPreloaded)
        ));

        firstSweep.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(firstSweep),
                        staticShot
                )
        );

        firstSweep.done().onTrue(
                sequence(
                        shootStatic(),
                        deadline(
                                waitUntil(superstructure.safeForTrench)
                                        .andThen(secondSweep.cmd())
                                        .asProxy(),
                                superstructure.runParametersWithHoodStowed(staticShot).asProxy()
                        )
                )
        );

        secondSweep.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(secondSweep),
                        staticShot
                )
        );

        secondSweep.done().onTrue(shootStatic());

        return routine;
    }

    //Right and Left ferry positions are swapped
    public AutoRoutine leftFerryClean() {
        final AutoRoutine routine = autoFactory.newRoutine("LeftFerryClean");
        final AutoTrajectory ferryAndClean = routine.trajectory("LeftFerryAndClean");

        routine.active().onTrue(parallel(
                runStartingTrajectory(ferryAndClean),
                runOnce(fuelState::setSimFuelPreloaded)
        ));

        routine.active().whileTrue(
                parallel(
                        intake.intake(),
                        sequence(
                                waitUntil(targetIsHub.negate()
                                        .and(turretSafe)),
                                ferryShootWhileMoving(FieldConstants.getFerryRight())
                                        .until(targetIsHub.or(turretSafe.negate())),
                                superstructure.runParametersWithHoodStowed(movingShot)
                                        .until(targetIsHub.and(turretSafe)),
                                shootWhileMoving()
                        ),
                        Commands.run(() -> Logger.recordOutput(LogKey + "TurretSafe", turretSafe))
                )
        );

        ferryAndClean.done().onTrue(
                swerve.runWheelXCommand()
        );

        return routine;
    }

    public AutoRoutine rightFerryClean() {
        final AutoRoutine routine = autoFactory.newRoutine("RightFerryClean");
        final AutoTrajectory ferryAndClean = routine.trajectory("RightFerryAndClean");

        routine.active().onTrue(runStartingTrajectory(ferryAndClean));

        routine.active().whileTrue(
                parallel(
                        intake.intake(),
                        sequence(
                                waitUntil(targetIsHub.negate()
                                        .and(turretSafe)),
                                ferryShootWhileMoving(FieldConstants.getFerryLeft())
                                        .until(targetIsHub.or(turretSafe.negate())),
                                superstructure.runParametersWithHoodStowed(movingShot)
                                        .until(targetIsHub.and(turretSafe)),
                                shootWhileMoving()
                        )
                )
        );

        ferryAndClean.done().onTrue(
                swerve.runWheelXCommand()
        );

        return routine;
    }

    public AutoRoutine leftDoubleSweepContinuous() {
        final AutoRoutine routine = autoFactory.newRoutine("LeftDoubleSweepContinuous");
        final AutoTrajectory firstSweep = routine.trajectory("LeftFirstSweepContinuous");
        final AutoTrajectory transition = routine.trajectory("LeftShootingTransition");
        final AutoTrajectory secondSweep = routine.trajectory("LeftSweepContinuous");

        routine.active().onTrue(parallel(
                runStartingTrajectory(firstSweep),
                runOnce(fuelState::setSimFuelPreloaded)
        ));

        firstSweep.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(firstSweep),
                        staticShot
                )
        );

        firstSweep.done().onTrue(
                transition.cmd()
        );

        transition.active().whileTrue(
                shootWhileMoving()
        );

        transition.done().onTrue(
                deadline(
                        secondSweep.cmd()
                                .asProxy(),
                        superstructure.runParametersWithHoodStowed(movingShot)
                                .asProxy()
                )
        );

        secondSweep.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(secondSweep),
                        staticShot
                )
        );

        secondSweep.done().onTrue(
                transition.cmd()
        );

        transition.active().whileTrue(
                shootWhileMoving()
        );

        transition.done().onTrue(
                deadline(
                        secondSweep.cmd()
                                .asProxy(),
                        superstructure.runParametersWithHoodStowed(movingShot)
                                .asProxy()
                )
        );

        return routine;
    }

    public AutoRoutine rightDoubleSweepContinuous() {
        final AutoRoutine routine = autoFactory.newRoutine("RightDoubleSweepContinuous");
        final AutoTrajectory firstSweep = routine.trajectory("RightFirstSweepContinuous");
        final AutoTrajectory transition = routine.trajectory("RightShootingTransition");
        final AutoTrajectory secondSweep = routine.trajectory("RightSweepContinuous");

        routine.active().onTrue(parallel(
                runStartingTrajectory(firstSweep),
                runOnce(fuelState::setSimFuelPreloaded)
        ));


        firstSweep.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(firstSweep),
                        staticShot
                )
        );

        firstSweep.done().onTrue(
                transition.cmd()
        );

        transition.active().whileTrue(
                shootWhileMoving()
        );

        transition.done().onTrue(
                deadline(
                        secondSweep.cmd()
                                .asProxy(),
                        superstructure.runParametersWithHoodStowed(movingShot)
                                .asProxy()
                )
        );

        secondSweep.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(secondSweep),
                        staticShot
                )
        );

        secondSweep.done().onTrue(
                transition.cmd()
        );

        transition.active().whileTrue(
                shootWhileMoving()
        );

        transition.done().onTrue(
                deadline(
                        secondSweep.cmd()
                                .asProxy(),
                        superstructure.runParametersWithHoodStowed(movingShot)
                                .asProxy()
                )
        );

        return routine;
    }

    private Command runStartingTrajectory(final AutoTrajectory startingTrajectory) {
        return Commands.sequence(
                startingTrajectory.resetOdometry(),
                startingTrajectory.cmd()
        ).withName("RunStartingTrajectory");
    }

    private Supplier<ShotParameters> staticParameters(final Supplier<Pose2d> robotPoseSupplier) {
        return staticShotProvider.parametersSupplier(
                robotPoseSupplier,
                superstructure::getRobotToTurret,
                swerve::getRobotRelativeSpeeds,
                FieldConstants::getHubPose
        );
    }

    private Supplier<ShotParameters> movingParameters(final Supplier<Pose2d> targetPoseSupplier) {
        return movingShotProvider.parametersSupplier(
                swerve::getPose,
                superstructure::getRobotToTurret,
                swerve::getRobotRelativeSpeeds,
                targetPoseSupplier
        );
    }

    private Supplier<ShotParameters> staticParametersFromPose(final Pose2d pose) {
        return staticParameters(() -> pose);
    }

    private Supplier<ShotParameters> staticParametersFromFinalPose(final AutoTrajectory trajectory) {
        return trajectory.getFinalPose()
                .map(this::staticParametersFromPose)
                .orElse(staticShot);
    }

    private Command intakeFromTrench(
            final Supplier<ShotParameters> fixed,
            final Supplier<ShotParameters> tracking
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
        return deadline(
                repeatingSequence(
                        waitUntil(robotStopped
                                .and(superstructure::atSetpoint)),
                        deadline(
                                indexer.feed()
                                        .onlyWhile(robotStopped
                                                .and(superstructure::atSetpoint)),
                                intake.stowFeed()
                        )
                )
                        .onlyWhile(fuelState.hasFuel),
                superstructure.runParameters(staticShot)
                        .onlyIf(turretSafe),
                swerve.runWheelXCommand()
        )
                .withName("ShootStatic");
    }

    private Command ferryShootWhileMoving(final Pose2d ferryPose) {
        return deadline(
                repeatingSequence(
                        waitUntil(superstructure::atSetpoint),
                        indexer.feed()
                                .onlyWhile(superstructure::atSetpoint)
                ).onlyWhile(fuelState.hasFuel
                        .or(intake.isIntaking)),
                superstructure.runParameters(movingParameters(() -> ferryPose))
        ).withName("FerryShotWhileMoving");
    }

    private Command shootWhileMoving() {
        return deadline(
                repeatingSequence(
                        waitUntil(superstructure::atSetpoint),
                        indexer.feed()
                                .onlyWhile(superstructure::atSetpoint)
                ),
                superstructure.runParameters(movingShot)
        ).withName("ShootWhileMoving");
    }
}