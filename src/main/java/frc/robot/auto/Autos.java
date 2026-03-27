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
import frc.robot.constants.PoseConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.calculation.MovingShot;
import frc.robot.subsystems.superstructure.calculation.ShotCalculation;
import frc.robot.subsystems.superstructure.calculation.StaticShot;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.utils.commands.trigger.LoggedTrigger;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class Autos {
    public static final String LogKey = "Auto";

    private final Swerve swerve;
    private final Intake intake;
    private final Indexer indexer;
    private final Superstructure superstructure;
    private final FuelState fuelState;

    private final AutoFactory autoFactory;

    private final Supplier<ShotCalculation> staticShotCalculation;
    private final Supplier<ShotCalculation> movingShotCalculation;

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

        this.staticShotCalculation = staticParameters(swerve::getPose);
        this.movingShotCalculation = MovingShot.getShotCalculationSupplier(
                swerve::getPose,
                swerve::getRobotRelativeSpeeds,
                getTargetPoseSupplier()
        );

        final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);
        this.robotStopped = group.t("RobotStopped",
                () -> ShootCommands.linearSpeed(swerve.getFieldRelativeSpeeds()) <= 0.01
        );
        this.targetIsHub = group.t("TargetIsHub",
                () -> ShootCommands.getTarget(swerve.getPose()) == ShootCommands.Target.HUB);
        this.turretSafe = group.t("TurretSafe",
                () -> {
                    final double safeXClose = FieldConstants.getTurretSafeXCloseBoundary();
                    final double safeXFar = FieldConstants.getTurretSafeXFarBoundary();
                    final double turretX = swerve.getPose().plus(PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D)
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
                        staticShotCalculation
                )
        );

        firstSweep.done().onTrue(
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
                        staticShotCalculation
                )
        );

        firstSweep.done().onTrue(
                sequence(
                        shootStatic(),
                        deadline(
                                waitUntil(superstructure::atSetpoint)
                                    .andThen(secondSweep.cmd())
                                    .asProxy(),
                                superstructure.runParametersWithHoodStowed(staticShotCalculation).asProxy()
                        )
                )
        );

        secondSweep.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(secondSweep),
                        staticShotCalculation
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
                        staticShotCalculation
                )
        );

        firstSweep.done().onTrue(
                sequence(
                        shootStatic(),
                        deadline(
                                waitUntil(superstructure.safeForTrench)
                                        .andThen(secondSweep.cmd())
                                        .asProxy(),
                                superstructure.runParametersWithHoodStowed(staticShotCalculation).asProxy()
                        )
                )
        );

        secondSweep.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(secondSweep),
                        staticShotCalculation
                )
        );

        secondSweep.done().onTrue(shootStatic());

        return routine;
    }

    public AutoRoutine leftFerryClean() {
        final AutoRoutine routine = autoFactory.newRoutine("LeftFerryClean");
        final AutoTrajectory ferryAndClean = routine.trajectory("LeftFerryAndClean");

        routine.active().onTrue(parallel(
                runStartingTrajectory(ferryAndClean),
                runOnce(fuelState::setSimFuelPreloaded)
        ));

        routine.active().whileTrue(
                Commands.parallel(
                        intake.intake(),
                        ferryShotWhileMoving(true)
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
                Commands.parallel(
                        intake.intake(),
                        ferryShotWhileMoving(false)
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
                        staticShotCalculation
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
                        superstructure.runParametersWithHoodStowed(movingShotCalculation)
                                .asProxy()
                )
        );

        secondSweep.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(secondSweep),
                        staticShotCalculation
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
                        superstructure.runParametersWithHoodStowed(movingShotCalculation)
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
                        staticShotCalculation
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
                        superstructure.runParametersWithHoodStowed(movingShotCalculation)
                                .asProxy()
                )
        );

        secondSweep.active().whileTrue(
                intakeFromTrench(
                        staticParametersFromFinalPose(secondSweep),
                        staticShotCalculation
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
                        superstructure.runParametersWithHoodStowed(movingShotCalculation)
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

    private Supplier<ShotCalculation> staticParameters(final Supplier<Pose2d> robotPoseSupplier) {
        return StaticShot.getShotCalculationSupplier(
                robotPoseSupplier,
                swerve::getRobotRelativeSpeeds,
                FieldConstants::getHubPose
        );
    }

    private Supplier<ShotCalculation> staticParametersFromPose(final Pose2d pose) {
        return staticParameters(() -> pose);
    }

    private Supplier<ShotCalculation> staticParametersFromFinalPose(final AutoTrajectory trajectory) {
        return trajectory.getFinalPose()
                .map(this::staticParametersFromPose)
                .orElse(staticShotCalculation);
    }

    private Command intakeFromTrench(
            final Supplier<ShotCalculation> fixed,
            final Supplier<ShotCalculation> tracking
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
                superstructure.runParameters(staticShotCalculation)
                        .onlyIf(turretSafe),
                swerve.runWheelXCommand()
        )
                .withName("ShootStatic");
    }

    private Command ferryShotWhileMoving(final boolean isLeftSideFerry) {
        return deadline(
                repeatingSequence(
                        waitUntil(superstructure::atSetpoint),
                        indexer.feed()
                                .onlyWhile(superstructure::atSetpoint)
                ).onlyWhile(fuelState.hasFuel
                        .or(intake.isIntaking)),
                superstructure.runParameters(MovingShot.getShotCalculationSupplier(
                        swerve::getPose,
                        swerve::getRobotRelativeSpeeds,
                        getTargetPoseSupplierWithFerryPose(
                                isLeftSideFerry ? FieldConstants.getFerryRight() : FieldConstants.getFerryLeft()
                        )
                ))
        ).withName("FerryShotWhileMoving");
    }

    private Command shootWhileMoving() {
        return deadline(
                repeatingSequence(
                        waitUntil(superstructure::atSetpoint),
                        indexer.feed()
                                .onlyWhile(superstructure::atSetpoint)
                ),
                superstructure.runParameters(movingShotCalculation)
        ).withName("ShootWhileMoving");
    }

    private Supplier<Pose2d> getTargetPoseSupplierWithFerryPose(final Pose2d ferryPose) {
        return () -> {
            final Pose2d robotPose = swerve.getPose();
            final ShootCommands.Target target = ShootCommands.getTarget(robotPose);
            return switch (target) {
                case HUB -> FieldConstants.getHubPose();
                case FERRY, FERRY_BLOCKED -> ferryPose;
            };
        };
    }

    private Supplier<Pose2d> getTargetPoseSupplier() {
        return () -> {
            final Pose2d robotPose = swerve.getPose();
            final ShootCommands.Target target = ShootCommands.getTarget(robotPose);
            return switch (target) {
                case HUB -> FieldConstants.getHubPose();
                case FERRY, FERRY_BLOCKED -> {
                    final boolean isRed = Robot.IsRedAlliance.getAsBoolean();
                    final Pose2d ferryLeft = FieldConstants.getFerryLeft();
                    final Pose2d ferryRight = FieldConstants.getFerryRight();

                    yield robotPose.getY() <= FieldConstants.getFerryLeftYBoundary()
                            ? (isRed ? ferryRight : ferryLeft)
                            : (isRed ? ferryLeft : ferryRight);
                }
            };
        };
    }
}