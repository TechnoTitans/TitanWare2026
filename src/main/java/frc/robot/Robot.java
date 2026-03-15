package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoChooser;
import frc.robot.auto.AutoOption;
import frc.robot.auto.Autos;
import frc.robot.constants.*;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.constants.SwerveConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.slide.IntakeSlide;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.superstructure.ShotCalculator;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.turret.Turret;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.commands.LoggedTrigger;
import frc.robot.utils.commands.RobotModeLoggedTriggers;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.logging.CommandLogger;
import frc.robot.utils.subsystems.VirtualSubsystem;
import frc.robot.utils.teleop.ControllerUtils;
import frc.robot.utils.teleop.SwerveSpeed;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.AllianceShift.AllianceShiftLogKey;

public class Robot extends LoggedRobot {
    private static final String LogKey = "Robot";
    private static final String AKitLogPath = "/U/logs";
    private static final String HootLogPath = "/U/logs";

    public static final BooleanSupplier IsRedAlliance = () -> {
        final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    };

    public final PowerDistribution powerDistribution = new PowerDistribution(
            HardwareConstants.PowerDistributionHub, PowerDistribution.ModuleType.kRev
    );

    public final Swerve swerve = new Swerve(
            Constants.CURRENT_MODE,
            SwerveConstants.CTRESwerve.DrivetrainConstants,
            new SwerveConstants.SwerveModuleConfig[]{
                    SwerveConstants.FrontLeftModule,
                    SwerveConstants.FrontRightModule,
                    SwerveConstants.BackLeftModule,
                    SwerveConstants.BackRightModule
            },
            SwerveConstants.CTRESwerve.FrontLeft,
            SwerveConstants.CTRESwerve.FrontRight,
            SwerveConstants.CTRESwerve.BackLeft,
            SwerveConstants.CTRESwerve.BackRight
    );

    public final PhotonVision photonVision = new PhotonVision(
            Constants.CURRENT_MODE,
            swerve
    );

    public final IntakeRoller intakeRoller = new IntakeRoller(
            Constants.CURRENT_MODE,
            HardwareConstants.INTAKE_ROLLER
    );

    public final IntakeSlide intakeSlide = new IntakeSlide(
            Constants.CURRENT_MODE,
            HardwareConstants.INTAKE_SLIDE
    );

    public final Feeder feeder = new Feeder(
            Constants.CURRENT_MODE,
            HardwareConstants.FEEDER
    );

    public final Hood hood = new Hood(
            Constants.CURRENT_MODE,
            HardwareConstants.HOOD
    );

    public final Turret turret = new Turret(
            Constants.CURRENT_MODE,
            HardwareConstants.TURRET,
            () -> -swerve.getFieldRelativeSpeeds().omegaRadiansPerSecond
    );

    public final Shooter shooter = new Shooter(
            Constants.CURRENT_MODE,
            HardwareConstants.SHOOTER
    );

    public final Spindexer spindexer = new Spindexer(
            Constants.CURRENT_MODE,
            HardwareConstants.SPINDEXER
    );

    private RobotCommands.ScoringMode scoringMode = RobotCommands.ScoringMode.Moving;

    private final Supplier<ShotCalculator.ShotCalculation> shotCalculationSupplier =
            () -> ShotCalculator.getShotCalculation(
                    swerve::getPose,
                    () -> scoringMode,
                    swerve::getFieldRelativeSpeeds
            );

    public final Superstructure superstructure = new Superstructure(
            turret,
            hood,
            shooter,
            shotCalculationSupplier
    );

    private final ComponentsSolver componentsSolver = new ComponentsSolver(
            turret::getTurretPosition,
            hood::getHoodPosition,
            intakeSlide::getIntakeSlidePositionRots
    );

    @SuppressWarnings("unused")
    private final FuelState fuelState = new FuelState(
            Constants.CURRENT_MODE,
            swerve,
            intakeRoller,
            feeder,
            superstructure,
            componentsSolver
    );

    private final RobotCommands robotCommands = new RobotCommands(
            swerve,
            intakeRoller,
            intakeSlide,
            superstructure,
            spindexer,
            feeder,
            () -> shotCalculationSupplier.get().target()
    );

    public final Autos autos = new Autos(
            swerve,
            superstructure,
            feeder,
            spindexer,
            intakeRoller,
            intakeSlide,
            photonVision
    );

    private final AutoChooser autoChooser = new AutoChooser(
//            new AutoOption(
//                    "DoNothing",
//                    autos::doNothing,
//                    Constants.CompetitionType.COMPETITION
//            )
            new AutoOption(
                    "RightCenterLineTrench",
                    autos::rightCenterLineTrench,
                    Constants.CompetitionType.COMPETITION
            )
    );

    public final CommandXboxController driverController = new CommandXboxController(RobotMap.MainController);
    public final CommandXboxController coController = new CommandXboxController(RobotMap.CoController);
    public final Alert driverControllerDisconnected = new Alert(
            "Driver controller not connected!",
            Alert.AlertType.kWarning
    );
    public final Alert coControllerDisconnected = new Alert(
            "Co controller not connected!",
            Alert.AlertType.kWarning
    );

    private final EventLoop teleopEventLoop = new EventLoop();
    private final EventLoop testEventLoop = new EventLoop();

    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    private final LoggedTrigger disabled = RobotModeLoggedTriggers.disabled(group);
    private final LoggedTrigger autonomousEnabled = RobotModeLoggedTriggers.autonomous(group);
    private final LoggedTrigger enabled = RobotModeLoggedTriggers.enabled(group);

    private static final double MatchTimeOffsetSeconds = 2;
    private final LoggedTrigger hubActive =
            group.t("HubActive", () -> AllianceShift.get(MatchTimeOffsetSeconds).hubStatus() == AllianceShift.HubStatus.ACTIVE);

    @Override
    public void robotInit() {
        if ((RobotBase.isReal() && Constants.CURRENT_MODE != Constants.RobotMode.REAL) ||
                (RobotBase.isSimulation() && Constants.CURRENT_MODE == Constants.RobotMode.REAL)) {
            DriverStation.reportWarning(
                    String.format(
                            "Potentially incorrect CURRENT_MODE \"%s\" specified, robot is running \"%s\"",
                            Constants.CURRENT_MODE,
                            RobotBase.getRuntimeType().toString()
                    ),
                    true
            );

            throw new RuntimeException("Incorrect CURRENT_MODE specified!");
        }

        // we never use LiveWindow, and apparently this causes loop overruns so disable it
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);

        // register shutdown hook
        ToClose.hook();

        // disable joystick not found warnings when in sim
        DriverStation.silenceJoystickConnectionWarning(Constants.CURRENT_MODE == Constants.RobotMode.SIM);

        switch (Constants.CURRENT_MODE) {
            case REAL -> {
                try {
                    Files.createDirectories(Paths.get(HootLogPath));
                    SignalLogger.setPath(HootLogPath);
                } catch (final IOException ioException) {
                    SignalLogger.setPath("/U");
                    DriverStation.reportError(
                            String.format(
                                    "Failed to create .hoot log path at \"%s\"! Falling back to default.\n%s",
                                    HootLogPath,
                                    ioException
                            ),
                            false
                    );
                }

                Logger.addDataReceiver(new WPILOGWriter(AKitLogPath));
                Logger.addDataReceiver(new NT4Publisher());
            }
            case SIM -> {
                // log to working directory when running sim
                // setPath doesn't seem to work in sim (path is ignored and hoot files are always sent to /logs)
//                SignalLogger.setPath("/logs");
                Logger.addDataReceiver(new WPILOGWriter("logs"));
                Logger.addDataReceiver(new NT4Publisher());

                DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
                DriverStationSim.notifyNewData();

                autonomousEnabled.whileTrue(
                        Commands.waitSeconds(20)
                                .andThen(() -> {
                                    DriverStationSim.setEnabled(false);
                                    DriverStationSim.notifyNewData();
                                })
                );
            }
            case REPLAY -> {
                setUseTiming(false);

                final String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(
                        new WPILOGWriter(
                                LogFileUtil.addPathSuffix(logPath, "_sim"),
                                WPILOGWriter.AdvantageScopeOpenBehavior.AUTO)
                );
            }
        }

        powerDistribution.clearStickyFaults();
        powerDistribution.setSwitchableChannel(true);

        configureStateTriggers();
        configureAutos();
        configureButtonBindings(teleopEventLoop);

        CommandLogger.init(CommandScheduler.getInstance());

        SignalLogger.enableAutoLogging(true);
        SignalLogger.start();
        ToClose.add(SignalLogger::stop);

        Logger.start();

        Logger.recordOutput("EmptyPose", Pose3d.kZero);

        final Pose3d[] emptyPoseArray = new Pose3d[6];
        Arrays.fill(emptyPoseArray, Pose3d.kZero);
        Logger.recordOutput("EmptyPoses", emptyPoseArray);
    }

    @Override
    public void robotPeriodic() {
        RefreshAll.refreshAll();

        CommandScheduler.getInstance().run();
        VirtualSubsystem.run();

        driverControllerDisconnected.set(!driverController.getHID().isConnected());
        coControllerDisconnected.set(!coController.getHID().isConnected());

        CommandLogger.periodic();
        Logger.recordOutput("ShotCalculation", shotCalculationSupplier.get());
        Logger.recordOutput("ScoringMode", scoringMode);

        componentsSolver.periodic();

        Logger.recordOutput("DistanceToHub", swerve.getPose()
                .transformBy(PoseConstants.Turret.ROBOT_TO_TURRET_TRANSFORM_2D)
                .getTranslation()
                .getDistance(FieldConstants.getHubTarget()));

        final AllianceShift allianceShift = AllianceShift.get(0);
        final AllianceShift offsetAllianceShift = AllianceShift.get(MatchTimeOffsetSeconds);
        Logger.recordOutput(AllianceShiftLogKey + "Normal", allianceShift);
        Logger.recordOutput(AllianceShiftLogKey + "Offset", offsetAllianceShift);
        Logger.recordOutput(AllianceShiftLogKey + "NormalHubStatus", allianceShift.hubStatus());
        Logger.recordOutput(AllianceShiftLogKey + "OffsetHubStatus", offsetAllianceShift.hubStatus());
    }

    @Override
    public void teleopInit() {
        //noinspection SuspiciousNameCombination
        swerve.setDefaultCommand(
                swerve.teleopDriveCommand(
                        driverController::getLeftY,
                        driverController::getLeftX,
                        driverController::getRightX
                )
        );
    }

    @Override
    public void teleopPeriodic() {
        teleopEventLoop.poll();

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

//        driverController.y(testEventLoop)
//                .whileTrue(swerve.linearTorqueCurrentSysIdQuasistaticCommand(SysIdRoutine.Direction.kForward));
//        driverController.a(testEventLoop)
//                .whileTrue(swerve.linearTorqueCurrentSysIdQuasistaticCommand(SysIdRoutine.Direction.kReverse));
//        driverController.x(testEventLoop)
//                .whileTrue(swerve.linearTorqueCurrentSysIdDynamicCommand(SysIdRoutine.Direction.kForward));
//        driverController.b(testEventLoop)
//                .whileTrue(swerve.linearTorqueCurrentSysIdDynamicCommand(SysIdRoutine.Direction.kReverse));

        driverController.x(testEventLoop).whileTrue(
                turret.voltageSysIdCommand()
                        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
        );

        driverController.y(testEventLoop).whileTrue(
                shooter.flywheelTorqueCurrentSysIdCommand()
                        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
        );

        driverController.leftBumper(testEventLoop).onTrue(Commands.runOnce(SignalLogger::stop));
    }

    @Override
    public void testPeriodic() {
        testEventLoop.poll();
    }

    @Override
    public void simulationPeriodic() {
    }

    public void configureStateTriggers() {
        enabled.onTrue(
                Commands.parallel(
                        superstructure.setGoalCommand(Superstructure.Goal.TRACKING),
                        intakeSlide.setGoalCommand(IntakeSlide.Goal.EXTEND)
                )
        );

        hubActive.onTrue(ControllerUtils.rumbleForDurationCommand(
                driverController.getHID(), GenericHID.RumbleType.kBothRumble, 1, 1
        ));

        disabled.onTrue(ControllerUtils.rumbleForDurationCommand(
                driverController.getHID(), GenericHID.RumbleType.kBothRumble, 0, 0.5
        ));

        disabled.onTrue(swerve.stopCommand());
    }

    public void configureAutos() {
        autonomousEnabled.whileTrue(Commands.deferredProxy(() -> autoChooser.getSelected().cmd()));

        autoChooser.addAutoOption(new AutoOption(
                "OnlyShootPreload",
                autos::onlyShootPreload,
                Constants.CompetitionType.COMPETITION
        ));

        autoChooser.addAutoOption(new AutoOption(
                "LeftCenterLineDepot",
                autos::leftCenterLineDepot,
                Constants.CompetitionType.COMPETITION
        ));

        autoChooser.addAutoOption(new AutoOption(
                "RightCenterLineOutpost",
                autos::rightCenterLineOutpost,
                Constants.CompetitionType.COMPETITION
        ));

//        autoChooser.addAutoOption(new AutoOption(
//                "RightCenterLineTrench",
//                autos::rightCenterLineTrench,
//                Constants.CompetitionType.COMPETITION
//        ));
    }

    public void configureButtonBindings(final EventLoop teleopEventLoop) {
        driverController.rightBumper(teleopEventLoop)
                .whileTrue(Commands.startEnd(
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.FAST),
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.NORMAL)
                ).withName("SwerveSpeedFast"));

        driverController.leftBumper(teleopEventLoop)
                .whileTrue(Commands.startEnd(
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.SLOW),
                        () -> SwerveSpeed.setSwerveSpeed(SwerveSpeed.Speeds.NORMAL)
                ).withName("SwerveSpeedSlow"));

        driverController.leftTrigger(0.5, teleopEventLoop).whileTrue(
                Commands.parallel(
                        intakeRoller.toGoal(IntakeRoller.Goal.INTAKE),
                        intakeSlide.toGoal(IntakeSlide.Goal.EXTEND)
                                .onlyIf(() -> intakeSlide.atGoal(IntakeSlide.Goal.EXTEND))
                )
        );

        driverController.rightTrigger(0.5, teleopEventLoop)
                .whileTrue(robotCommands.shootWhileMoving());

        coController.leftTrigger(0.5, teleopEventLoop).whileTrue(
                intakeRoller.toGoal(IntakeRoller.Goal.INTAKE)
        );

        coController.y(teleopEventLoop).onTrue(robotCommands.deployIntake());

        coController.a(teleopEventLoop).onTrue(robotCommands.stowIntake());

        coController.rightTrigger(0.5, teleopEventLoop).whileTrue(
                Commands.parallel(
                        Commands.runOnce(() -> scoringMode = RobotCommands.ScoringMode.Stationary),
                        robotCommands.shootStationary()
                ).finallyDo(() -> scoringMode = RobotCommands.ScoringMode.Moving)
        );

        coController.x(teleopEventLoop).whileTrue(
                robotCommands.shootNoCheck()
        );

        coController.leftBumper(teleopEventLoop).whileTrue(
                robotCommands.shootSuperstructureZero()
        );
    }
}
