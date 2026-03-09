package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

public class Autos {
    public static final String LogKey = "Auto";

    private final Swerve swerve;
    private final Superstructure superstructure;
    private final Feeder feeder;
    private final AutoFactory autoFactory;

    public Autos(
            final Swerve swerve,
            final Superstructure superstructure,
            final Feeder feeder,
            final PhotonVision photonVision
    ) {
        this.swerve = swerve;
        this.superstructure = superstructure;
        this.feeder = feeder;

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
    }

    private Command runStartingTrajectory(final AutoTrajectory startingTrajectory) {
        return Commands.sequence(
                startingTrajectory.resetOdometry(),
                startingTrajectory.cmd()
        );
    }

    public AutoRoutine doNothing() {
        final AutoRoutine routine = autoFactory.newRoutine("DoNothing");

        routine.active().whileTrue(
                Commands.waitUntil(RobotModeTriggers.autonomous().negate())
        );

        return routine;
    }


    //TODO: Move modified shooting stuff to robot commands?
    public AutoRoutine leftCenterLineDepot() {
        final AutoRoutine routine = autoFactory.newRoutine("LeftCenterLineDepot");
        final AutoTrajectory startToCenterLineAndBack = routine.trajectory("LeftStartToCenterLineAndBack");
        final AutoTrajectory shootingToDepot = routine.trajectory("LeftShootingToDepot");
        final AutoTrajectory depotToShooting = routine.trajectory("LeftDepotToShooting");

        routine.active().onTrue(runStartingTrajectory(startToCenterLineAndBack));

        startToCenterLineAndBack.atTimeBeforeEnd(0.3).onTrue(
                superstructure.setGoal(Superstructure.Goal.SHOOTING)
        );

        startToCenterLineAndBack.done().onTrue(
                Commands.sequence(
                        Commands.parallel(
                                swerve.runWheelXCommand(),
                                feeder.toGoal(Feeder.Goal.FEED)
                        ).withTimeout(4),
                        shootingToDepot.cmd()
                )
        );

        shootingToDepot.done().onTrue(
                depotToShooting.cmd()
        );

        depotToShooting.done().onTrue(
                Commands.parallel(
                        swerve.runWheelXCommand(),
                        feeder.toGoal(Feeder.Goal.FEED)
                ).withTimeout(7)
        );

        routine.active().whileTrue(
                Commands.waitUntil(RobotModeTriggers.autonomous().negate())
        );


        return routine;
    }

    public AutoRoutine rightCenterLineOutpost() {
        final AutoRoutine routine = autoFactory.newRoutine("RightCenterLineOutpost");
        final AutoTrajectory startToCenterLineAndBack = routine.trajectory("RightStartToCenterLineAndBack");
        final AutoTrajectory shootingToOutpost = routine.trajectory("RightShootingToOutput");
        final AutoTrajectory outpostToShooting = routine.trajectory("RightOutpostToShooting");

        routine.active().onTrue(runStartingTrajectory(startToCenterLineAndBack));

        startToCenterLineAndBack.atTimeBeforeEnd(0.3).onTrue(
                superstructure.setGoal(Superstructure.Goal.SHOOTING)
        );

        startToCenterLineAndBack.done().onTrue(
                Commands.sequence(
                        Commands.parallel(
                                swerve.runWheelXCommand(),
                                Commands.sequence(
//                                        Commands.waitUntil(superstructure.atSetpoint),
                                        feeder.toGoal(Feeder.Goal.FEED)
                                )
                        ).withTimeout(4),
                        shootingToOutpost.cmd()
                )
        );

        shootingToOutpost.done().onTrue(
                Commands.sequence(
                        Commands.waitSeconds(3),
                        outpostToShooting.cmd()
                )
        );

        outpostToShooting.done().onTrue(
                Commands.parallel(
                        swerve.runWheelXCommand(),
//                        Commands.waitUntil(superstructure.atSetpoint),
                        feeder.toGoal(Feeder.Goal.FEED)
                ).withTimeout(4)
        );

        routine.active().whileTrue(
                Commands.waitUntil(RobotModeTriggers.autonomous().negate())
        );


        return routine;
    }
}