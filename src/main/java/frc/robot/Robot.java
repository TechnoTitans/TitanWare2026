package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoChooser;
import frc.robot.auto.AutoOption;
import frc.robot.auto.Autos;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.FuelState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.constants.SwerveConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.feeder.Feeder;
import frc.robot.subsystems.indexer.spindexer.Spindexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.intake.slide.IntakeSlide;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.turret.Turret;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.commands.CommandsExt;
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
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class Robot extends LoggedRobot {
    protected static final String LogKey = "Robot";
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
            Constants.RobotMode.DISABLED,
            swerve
    );

    public final Turret turret = new Turret(
            Constants.CURRENT_MODE,
            HardwareConstants.TURRET_CONSTANTS
    );
    public final Shooter shooter = new Shooter(
            Constants.CURRENT_MODE,
            HardwareConstants.SHOOTER_CONSTANTS
    );
    public final Hood hood = new Hood(
            Constants.CURRENT_MODE,
            HardwareConstants.HOOD_CONSTANTS
    );
    public final Superstructure superstructure = new Superstructure(
            turret, shooter, hood
    );

    public final Spindexer spindexer = new Spindexer(Constants.CURRENT_MODE, HardwareConstants.SPINDEXER_CONSTANTS);
    public final Feeder feeder = new Feeder(Constants.CURRENT_MODE, HardwareConstants.FEEDER_CONSTANTS);
    public final Indexer indexer = new Indexer(spindexer, feeder);

    public final IntakeSlide intakeSlide = new IntakeSlide(
            Constants.CURRENT_MODE,
            HardwareConstants.INTAKE_SLIDE_CONSTANTS
    );
    public final IntakeRollers intakeRollers = new IntakeRollers(
            Constants.CURRENT_MODE,
            HardwareConstants.INTAKE_CONSTANTS
    );
    public final Intake intake = new Intake(intakeSlide, intakeRollers);

    public final FuelState fuelState = new FuelState(Constants.CURRENT_MODE, swerve, intake, indexer, superstructure);
    public final ShootCommands shootCommands = new ShootCommands(
            swerve, intake, indexer, fuelState, superstructure
    );

    public final Autos autos = new Autos(
            swerve, intake, indexer, fuelState, superstructure,
            photonVision
    );
    public final AutoChooser autoChooser = new AutoChooser(
            new AutoOption(
                    "DoNothing",
                    autos::doNothing,
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
    private final LoggedTrigger teleopEnabled = RobotModeLoggedTriggers.teleop(group);
    private final LoggedTrigger autonomousEnabled = RobotModeLoggedTriggers.autonomous(group);
    private final LoggedTrigger endgameTrigger = group.t("endgame", () -> DriverStation.getMatchTime() <= 20)
            .and(DriverStation::isFMSAttached)
            .and(teleopEnabled);

    private final LoggedTrigger hubActive =
            group.t("HubActive", () -> AllianceShift.get().hubStatus() == HubStatus.ACTIVE);

    public Robot() {
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
        DriverStation.silenceJoystickConnectionWarning(Constants.CURRENT_MODE != Constants.RobotMode.REAL);

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
    }

    @Override
    public void robotPeriodic() {
        RefreshAll.refreshAll();

        CommandScheduler.getInstance().run();
        CommandLogger.periodic();

        VirtualSubsystem.run();

        logComponentPoses();

        driverControllerDisconnected.set(!driverController.getHID().isConnected());
        coControllerDisconnected.set(!coController.getHID().isConnected());

        Logger.recordOutput(
                "DistanceToHub",
                FieldConstants.getHubPose()
                        .getTranslation()
                        .getDistance(superstructure.getTurretTranslation(swerve.getPose()))
        );

        final AllianceShift allianceShift = AllianceShift.get();
        Logger.recordOutput("AllianceShift", allianceShift);
        Logger.recordOutput("HubStatus", allianceShift.hubStatus());
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousPeriodic() {}

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

        driverController.leftBumper(testEventLoop).onTrue(Commands.runOnce(SignalLogger::stop));
        driverController.a(testEventLoop).whileTrue(
                swerve.wheelRadiusCharacterization()
        );
    }

    @Override
    public void testPeriodic() {
        testEventLoop.poll();
    }

    @Override
    public void simulationPeriodic() {}

    public void logComponentPoses() {
        final Pose3d[] superstructurePoses = superstructure.getComponentPoses();
        final Pose3d[] intakeSlidePoses = intakeSlide.getComponentPoses();
        Logger.recordOutput(
                "Components",
                superstructurePoses[0],
                intakeSlidePoses[1],
                intakeSlidePoses[0],
                superstructurePoses[1]
        );
    }

    public void configureStateTriggers() {
        teleopEnabled.whileTrue(CommandsExt.defaultCommand(shootCommands.trackTarget()));

        autonomousEnabled.or(teleopEnabled).onTrue(intake.deploy());

        hubActive.onTrue(ControllerUtils.rumbleForDurationCommand(
                driverController.getHID(), GenericHID.RumbleType.kBothRumble, 0.5, 1
        ));

        disabled.onTrue(swerve.stopCommand());
    }

    public void configureAutos() {
        autonomousEnabled.whileTrue(Commands.deferredProxy(() -> autoChooser.getSelected().cmd()));

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

        driverController.rightTrigger(0.5, teleopEventLoop)
                .whileTrue(shootCommands.shoot())
                .onFalse(intake.deploy());

        driverController.leftTrigger(0.5, teleopEventLoop).whileTrue(intake.intake());

        coController.y(teleopEventLoop).onTrue(intake.deploy());

        coController.a(teleopEventLoop).onTrue(intake.stow());

        coController.rightTrigger(0.5, teleopEventLoop)
                .whileTrue(shootCommands.stopAndShoot())
                .onFalse(intake.deploy());
    }

    public enum HubStatus {
        ACTIVE,
        INACTIVE,
        UNKNOWN
    }

    public enum AllianceShift {
        AUTO,
        TRANSITION,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        ENDGAME,
        UNKNOWN;

        public HubStatus hubStatus() {
            if (this == AUTO || this == TRANSITION || this == ENDGAME) {
                return HubStatus.ACTIVE;
            } else if (this == UNKNOWN) {
                return HubStatus.UNKNOWN;
            }

            final String gameMessage = DriverStation.getGameSpecificMessage();
            if (gameMessage.isEmpty()) {
                return HubStatus.UNKNOWN;
            }

            final Optional<DriverStation.Alliance> maybeAlliance = DriverStation.getAlliance();
            if (maybeAlliance.isEmpty()) {
                return HubStatus.UNKNOWN;
            }

            final DriverStation.Alliance alliance = maybeAlliance.get();
            final DriverStation.Alliance autoWinningAlliance;
            switch (gameMessage.charAt(0)) {
                case 'R' -> autoWinningAlliance = DriverStation.Alliance.Red;
                case 'B' -> autoWinningAlliance = DriverStation.Alliance.Blue;
                default -> {
                    return HubStatus.UNKNOWN;
                }
            }

            final boolean wonAuto = alliance == autoWinningAlliance;
            return switch (this) {
                case SHIFT_1, SHIFT_3 -> wonAuto ? HubStatus.INACTIVE : HubStatus.ACTIVE;
                case SHIFT_2, SHIFT_4 -> wonAuto ? HubStatus.ACTIVE : HubStatus.INACTIVE;
                default -> HubStatus.UNKNOWN;
            };
        }

        public static AllianceShift get() {
            if (DriverStation.isAutonomousEnabled()) {
                return AUTO;
            } else if (DriverStation.isDisabled()) {
                return UNKNOWN;
            }

            final double matchTime = DriverStation.getMatchTime();
            if (DriverStation.isFMSAttached()) {
                if (matchTime > 130) {
                    return TRANSITION;
                } else if (matchTime > 105) {
                    return SHIFT_1;
                } else if (matchTime > 80) {
                    return SHIFT_2;
                } else if (matchTime > 55) {
                    return SHIFT_3;
                } else if (matchTime > 30) {
                    return SHIFT_4;
                } else {
                    return ENDGAME;
                }
            } else {
                if (matchTime > 110) {
                    return ENDGAME;
                } else if (matchTime > 85) {
                    return SHIFT_4;
                } else if (matchTime > 60) {
                    return SHIFT_3;
                } else if (matchTime > 35) {
                    return SHIFT_2;
                } else if (matchTime > 10) {
                    return SHIFT_1;
                } else {
                    return TRANSITION;
                }
            }
        }
    }
}
