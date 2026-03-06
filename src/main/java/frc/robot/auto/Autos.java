package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Robot;
import frc.robot.RobotCommands;
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
    private final RobotCommands robotCommands;

    public Autos(
            final Swerve swerve,
            final Superstructure superstructure,
            final Feeder feeder,
            final PhotonVision photonVision,
            final RobotCommands robotCommands
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

        this.robotCommands = robotCommands;
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

    public AutoRoutine RightCenterLineDepot() {
        final AutoRoutine routine = autoFactory.newRoutine("RightCenterLineDepot");
        final AutoTrajectory startToCenterLineAndBack = routine.trajectory("RightStartToCenterLineAndBack");
        final AutoTrajectory zoneLineToDepot = routine.trajectory("RightZoneLineToDepot");
        final AutoTrajectory depotToScoring = routine.trajectory("RightDepotToScoring");

        routine.active().onTrue(runStartingTrajectory(startToCenterLineAndBack));

        startToCenterLineAndBack.atTimeBeforeEnd(0.3).onTrue(
                superstructure.setGoal(Superstructure.Goal.SHOOTING)
        );

        startToCenterLineAndBack.done().onTrue(
                Commands.sequence(
                        Commands.parallel(
                                swerve.runWheelXCommand(),
                                feeder.toGoal(Feeder.Goal.FEED)
                        ).withTimeout(7),
                        zoneLineToDepot.cmd()
                )
        );

        zoneLineToDepot.done().onTrue(
                Commands.sequence(
                        Commands.waitSeconds(3),
                        depotToScoring.cmd()
                )
        );

        depotToScoring.done().onTrue(
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
}