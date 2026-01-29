// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.constants.SwerveConstants;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.slide.IntakeSlide;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.superstructure.ShotCalculator;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.feeder.Feeder;
import frc.robot.subsystems.superstructure.hood.Hood;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import frc.robot.subsystems.superstructure.turret.Turret;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.logging.LoggedCommandScheduler;
import frc.robot.utils.solver.ComponentsSolver;
import frc.robot.utils.subsystems.VirtualSubsystem;
import frc.robot.utils.teleop.ControllerUtils;
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
import java.util.function.Supplier;

public class Robot extends LoggedRobot {
    private static final String AKitLogPath = "/U/logs";
    private static final String HootLogPath = "/U/logs";

    public static final BooleanSupplier IsRedAlliance = () -> {
        final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    };

    public final PowerDistribution powerDistribution = new PowerDistribution(
            RobotMap.PowerDistributionHub, PowerDistribution.ModuleType.kRev
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

    public final Turret turret = new Turret(
            Constants.CURRENT_MODE,
            HardwareConstants.TURRET
    );

    public final Hood hood = new Hood(
            Constants.CURRENT_MODE,
            HardwareConstants.HOOD
    );

    public final Shooter shooter = new Shooter(
            Constants.CURRENT_MODE,
            HardwareConstants.SHOOTER
    );

    public final Spindexer spindexer = new Spindexer(
            Constants.CURRENT_MODE,
            HardwareConstants.SPINDEXER
    );

    //TODO: Change to Moving when SOTM is implemented
    private RobotCommands.ScoringMode scoringMode =
            RobotCommands.ScoringMode.Stationary;

    private final Supplier<ShotCalculator.ShotCalculation> shotCalculationSupplier =
            () -> ShotCalculator.getShotCalculation(
                    swerve::getPose,
                    () -> scoringMode
            );


    public final Superstructure superstructure = new Superstructure(
            feeder,
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

    private final RobotCommands robotCommands = new RobotCommands(
            swerve,
            intakeRoller,
            intakeSlide,
            superstructure,
            spindexer
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

    private final Trigger disabled = RobotModeTriggers.disabled();
    public final Trigger autonomousEnabled = RobotModeTriggers.autonomous();
    public final Trigger teleopEnabled = RobotModeTriggers.teleop();
    private final Trigger endgameTrigger = new Trigger(() -> DriverStation.getMatchTime() <= 20)
            .and(DriverStation::isFMSAttached)
            .and(RobotModeTriggers.teleop());


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
                Logger.addDataReceiver(new WPILOGWriter(""));
                Logger.addDataReceiver(new NT4Publisher());

                DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
                DriverStationSim.notifyNewData();

                autonomousEnabled.whileTrue(
                        Commands.waitSeconds(15)
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

        LoggedCommandScheduler.init(CommandScheduler.getInstance());

        SignalLogger.enableAutoLogging(true);
        SignalLogger.start();
        ToClose.add(SignalLogger::stop);

        Logger.start();

        Logger.recordOutput("EmptyPose", Pose3d.kZero);
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);
        RefreshAll.refreshAll();

        CommandScheduler.getInstance().run();
        VirtualSubsystem.run();

        driverControllerDisconnected.set(!driverController.getHID().isConnected());
        coControllerDisconnected.set(!coController.getHID().isConnected());

        LoggedCommandScheduler.periodic();
        Logger.recordOutput("ShotCalculation", shotCalculationSupplier.get());
        Logger.recordOutput("ScoringType", scoringMode);
        componentsSolver.periodic();
        robotCommands.periodic();

        Threads.setCurrentThreadPriority(false, 10);
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
    }

    @Override
    public void testPeriodic() {
        testEventLoop.poll();
    }

    @Override
    public void simulationPeriodic() {}

    public void configureStateTriggers() {
        autonomousEnabled.onTrue(
                Commands.sequence(
                        Commands.parallel(
                                hood.home(),
                                intakeSlide.home()
                        ),
                        Commands.parallel(
                                intakeSlide.setGoal(IntakeSlide.Goal.INTAKE),
                                intakeRoller.setGoal(IntakeRoller.Goal.INTAKE)
                        )
                )
        );

        teleopEnabled.onTrue(
                Commands.parallel(
                        intakeSlide.setGoal(IntakeSlide.Goal.INTAKE),
                        intakeRoller.setGoal(IntakeRoller.Goal.INTAKE)
                )
        );

        endgameTrigger.onTrue(ControllerUtils.rumbleForDurationCommand(
                driverController.getHID(), GenericHID.RumbleType.kBothRumble, 0.5, 1)
        );

        disabled.onTrue(swerve.stopCommand());
    }

    public void configureAutos() {

    }

    public void configureButtonBindings(final EventLoop teleopEventLoop) {
        //TODO: Might be to complex
        driverController.rightTrigger(0.5, teleopEventLoop).whileTrue(
                robotCommands.shoot(() -> scoringMode)

        );

        driverController.leftTrigger(0.5, teleopEventLoop).whileTrue(
                robotCommands.manualIntake()
        );

        driverController.povUp().onTrue(
                Commands.runOnce(() -> this.scoringMode = RobotCommands.ScoringMode.Stationary)
        );

        driverController.povLeft().onTrue(
                Commands.runOnce(() -> this.scoringMode = RobotCommands.ScoringMode.Turret_Off)
        );

        driverController.povDown().onTrue(
                Commands.runOnce(() -> this.scoringMode = RobotCommands.ScoringMode.Moving)
        );

        driverController.povRight().onTrue(
                intakeSlide.setGoal(IntakeSlide.Goal.STOW)
        );
    }
}
