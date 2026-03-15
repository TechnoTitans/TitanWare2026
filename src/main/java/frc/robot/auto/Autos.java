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
import frc.robot.RobotCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.slide.IntakeSlide;
import frc.robot.subsystems.spindexer.Spindexer;
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
    private final Superstructure superstructure;
    private final Feeder feeder;
    private final Spindexer spindexer;
    private final IntakeRoller intakeRoller;
    private final IntakeSlide intakeSlide;

    private final AutoFactory autoFactory;

    private final Supplier<ShotCalculator.ShotCalculation> staticShot;

    private final LoggedTrigger robotStopped;
    private final LoggedTrigger turretSafe;

    public Autos(
            final Swerve swerve,
            final Superstructure superstructure,
            final Feeder feeder,
            final Spindexer spindexer,
            final IntakeRoller intakeRoller,
            final IntakeSlide intakeSlide,
            final PhotonVision photonVision
    ) {
        this.swerve = swerve;
        this.superstructure = superstructure;
        this.feeder = feeder;
        this.spindexer = spindexer;
        this.intakeRoller = intakeRoller;
        this.intakeSlide = intakeSlide;

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

        this.staticShot = staticParameters();

        LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);
        this.robotStopped = group.t("RobotStopped",
                () -> RobotCommands.linearSpeed(swerve.getFieldRelativeSpeeds()) <= 0.01
        );
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

    private Supplier<ShotCalculator.ShotCalculation> staticParameters() {
        return () -> ShotCalculator.getShotCalculation(
                swerve::getPose,
                () -> RobotCommands.ScoringMode.Stationary,
                swerve::getFieldRelativeSpeeds
        );
    }

    private Supplier<ShotCalculator.ShotCalculation> staticParametersFromPose(final Pose2d pose) {
        return staticParameters();
    }

    private Supplier<ShotCalculator.ShotCalculation> staticParametersFromFinalPose(final AutoTrajectory trajectory) {
        return trajectory.getFinalPose()
                .map(this::staticParametersFromPose)
                .orElse(staticShot);
    }

    private Command shootStatic() {
        final Timer timer = new Timer();

        return deadline(
                repeatingSequence(
                        waitUntil(robotStopped
                                .and(superstructure.atSetpoint)),
                        runOnce(timer::start),
                        feeder.toGoal(Feeder.Goal.FEED)
                                .onlyWhile(robotStopped
                                        .and(superstructure.atSetpoint))
                                .finallyDo(timer::stop)
                ).until(() -> timer.hasElapsed(SHOOTING_TIME)),
                superstructure.toGoal(Superstructure.Goal.SHOOTING),
                intakeSlide.toGoal(IntakeSlide.Goal.SHOOTING),
                spindexer.toGoal(Spindexer.Goal.FEED),
                swerve.runWheelXCommand(),
                Commands.run(
                        () -> Logger.recordOutput("RobotStopped", robotStopped)
                ),
                runOnce(timer::reset)
        )
                .withName("ShootStatic");
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

        final ShotCalculator.ShotCalculation firstShotCalculation = ShotCalculator.getShotCalculationFromPose(
                startToCenterLineAndBack.getFinalPose().orElse(Pose2d.kZero)
        );

        final ShotCalculator.ShotCalculation secondShotCalculation = ShotCalculator.getShotCalculationFromPose(
                shootingToDepot.getFinalPose().orElse(Pose2d.kZero)
        );

        routine.active().onTrue(
                Commands.parallel(
                        runStartingTrajectory(startToCenterLineAndBack),
                        intakeRoller.setGoal(IntakeRoller.Goal.INTAKE),
                        Commands.sequence(
                                superstructure.setGoalCommand(Superstructure.Goal.STATIC_SHOT_PREP),
                                Commands.runOnce(() -> superstructure.updateStaticShotParameter(firstShotCalculation))
                        )
                ).withName("StartCenterLine")
        );

        startToCenterLineAndBack.done().onTrue(
                Commands.parallel(
                        intakeRoller.setGoal(IntakeRoller.Goal.STOP),
                        sequence(
                                shootStatic(),
                                superstructure.setGoalCommand(Superstructure.Goal.STATIC_SHOT_PREP),
                                Commands.runOnce(() -> superstructure.updateStaticShotParameter(secondShotCalculation)),
                                shootingToDepot.cmd()
                        )
                ).withName("CenterLineShoot")
        );

        shootingToDepot.done().onTrue(shootStatic().withName("ShootFromDepot"));

        return routine;
    }

    public AutoRoutine rightCenterLineOutpost() {
        final AutoRoutine routine = autoFactory.newRoutine("RightCenterLineOutpost");
        final AutoTrajectory startToCenterLineAndBack = routine.trajectory("RightStartToCenterLineAndBack");
        final AutoTrajectory shootingToOutpost = routine.trajectory("RightShootingToOutput");

        final ShotCalculator.ShotCalculation firstShotCalculation = ShotCalculator.getShotCalculationFromPose(
                startToCenterLineAndBack.getFinalPose().orElse(Pose2d.kZero)
        );

        final ShotCalculator.ShotCalculation secondShotCalculation = ShotCalculator.getShotCalculationFromPose(
                shootingToOutpost.getFinalPose().orElse(Pose2d.kZero)
        );

        routine.active().onTrue(
                Commands.parallel(
                        runStartingTrajectory(startToCenterLineAndBack),
                        intakeRoller.setGoal(IntakeRoller.Goal.INTAKE),
                        Commands.sequence(
                                superstructure.setGoalCommand(Superstructure.Goal.STATIC_SHOT_PREP),
                                Commands.runOnce(() -> superstructure.updateStaticShotParameter(firstShotCalculation))
                        )
                ).withName("StartCenterLine")
        );

        startToCenterLineAndBack.done().onTrue(
                Commands.parallel(
                        intakeRoller.setGoal(IntakeRoller.Goal.STOP),
                        sequence(
                                shootStatic(),
                                superstructure.setGoalCommand(Superstructure.Goal.STATIC_SHOT_PREP),
                                Commands.runOnce(() -> superstructure.updateStaticShotParameter(secondShotCalculation)),
                                shootingToOutpost.cmd()
                        )
                ).withName("CenterLineShoot")

        );

        shootingToOutpost.done().onTrue(shootStatic().withName("ShootFromOutpost"));

        return routine;
    }

    public AutoRoutine rightDoubleSweep() {
        final AutoRoutine routine = autoFactory.newRoutine("RightDoubleSweep");
        final AutoTrajectory startToCenterLineAndBack = routine.trajectory("RightStartToCenterLineAndBack");
        final AutoTrajectory rightSweep = routine.trajectory("RightSweep");

        final ShotCalculator.ShotCalculation firstShotCalculation = ShotCalculator.getShotCalculationFromPose(
                startToCenterLineAndBack.getFinalPose().orElse(Pose2d.kZero)
        );

        final ShotCalculator.ShotCalculation secondShotCalculation = ShotCalculator.getShotCalculationFromPose(
                rightSweep.getFinalPose().orElse(Pose2d.kZero)
        );

        routine.active().onTrue(
                Commands.parallel(
                        runStartingTrajectory(startToCenterLineAndBack),
                        intakeRoller.setGoal(IntakeRoller.Goal.INTAKE),
                        Commands.sequence(
                                superstructure.setGoalCommand(Superstructure.Goal.STATIC_SHOT_PREP),
                                Commands.runOnce(() -> superstructure.updateStaticShotParameter(firstShotCalculation))
                        )
                ).withName("StartCenterLine")
        );

        startToCenterLineAndBack.done().onTrue(
                Commands.parallel(
                        sequence(
                                shootStatic(),
                                superstructure.setGoalCommand(Superstructure.Goal.STATIC_SHOT_PREP),
                                Commands.runOnce(() -> superstructure.updateStaticShotParameter(secondShotCalculation)),
                                rightSweep.cmd()
                        )
                ).withName("CenterLineShoot")
        );

        rightSweep.done().onTrue(shootStatic().withName("ShootFromTrench"));

        return routine;
    }
}