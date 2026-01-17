package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.swerve.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
//import frc.robot.auto.Autos;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.SwerveIO.SwerveDriveState;
import frc.robot.subsystems.drive.constants.SwerveConstants;
import frc.robot.subsystems.drive.controllers.HolonomicChoreoController;
import frc.robot.subsystems.drive.controllers.HolonomicDriveController;
import frc.robot.utils.commands.LoggedTrigger;
import frc.robot.utils.gyro.GyroUtils;
import frc.robot.utils.teleop.ControllerUtils;
import frc.robot.utils.teleop.SwerveSpeed;
import org.littletonrobotics.junction.Logger;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.constants.SwerveConstants.Config;

public class Swerve extends SubsystemBase {
    protected static final String LogKey = "Swerve";
    protected static final String OdometryLogKey = LogKey + "/Odometry";
    private static final double OdometryBufferHistorySeconds = 1.5;

    private final Constants.RobotMode mode;
    private final LoggedTrigger.Group group;
    private final LoggedTrigger allowedToChangeForwardDirection;

    private final SwerveIO swerveIO;
    private final SwerveIOInputsAutoLogged inputs;
    private final ModuleIOInputsAutoLogged[] moduleInputs;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator replayPoseEstimator;
    private boolean replayPoseEstimatorReset = false;

    private boolean stateLastValid = false;
    private final List<Consumer<SwerveDriveState>> onStateValidCallbacks = new ArrayList<>();

    private final LinearFilter odometryPeriodFilter = LinearFilter.movingAverage(20);
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
            TimeInterpolatableBuffer.createBuffer(OdometryBufferHistorySeconds);

    private final SwerveRequest.FieldCentric driveFieldRelative = new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position)
            .withDesaturateWheelSpeeds(true)
            .withCenterOfRotation(Config.centerOfRotationMeters());

    private final SwerveRequest.RobotCentric driveRobotRelative = new SwerveRequest.RobotCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position)
            .withDesaturateWheelSpeeds(true)
            .withCenterOfRotation(Config.centerOfRotationMeters());

    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position)
            .withDesaturateWheelSpeeds(true)
            .withCenterOfRotation(Config.centerOfRotationMeters());

    private final SwerveRequest.SysIdSwerveTranslation sysIdTranslationVoltage =
            new SwerveRequest.SysIdSwerveTranslation();

    private final RequestSysIdSwerveRotationVoltage sysIdSwerveRotationVoltage =
            new RequestSysIdSwerveRotationVoltage();

    private final RequestSysIdSwerveTranslationTorqueCurrent sysIdTranslationTorqueCurrent =
            new RequestSysIdSwerveTranslationTorqueCurrent();

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.Position);

    public final LoggedTrigger atHeadingSetpoint;
    private boolean headingControllerActive = false;
    private Rotation2d headingTarget = Rotation2d.kZero;
    private final PIDController headingController;

    public final LoggedTrigger atHolonomicDrivePose;
    public final LoggedTrigger atHolonomicDrivePoseStopped;
    private boolean holonomicControllerActive = false;
    private Pose2d holonomicPoseTarget = Pose2d.kZero;
    private final HolonomicDriveController holonomicDriveController;
    private final PIDController holdAxisPID;

    private final HolonomicChoreoController choreoController;
    private Rotation2d appliedForwardDirection;

    private final SysIdRoutine linearVoltageSysIdRoutine;
    private final SysIdRoutine linearTorqueCurrentSysIdRoutine;
    private final SysIdRoutine angularVoltageSysIdRoutine;

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }

    public enum DriveAxis {X, Y}

    @SafeVarargs
    public Swerve(
            final Constants.RobotMode mode,
            final SwerveDrivetrainConstants drivetrainConstants,
            final SwerveConstants.SwerveModuleConfig[] moduleConfigs,
            final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... moduleConstants
    ) {
        this.mode = mode;
        this.group = LoggedTrigger.Group.from(LogKey);
        this.allowedToChangeForwardDirection =
                group.t("AllowedToChangeForwardDirection", RobotModeTriggers.disabled());

        this.swerveIO = switch (mode) {
            case REAL -> new SwerveIOReal(drivetrainConstants, moduleConstants);
            case SIM -> new SwerveIOSim(drivetrainConstants, moduleConstants);
            case REPLAY, DISABLED -> new SwerveIO() {};
        };
        this.inputs = new SwerveIOInputsAutoLogged();
        this.moduleInputs = new ModuleIOInputsAutoLogged[moduleConfigs.length];
        for (int i = 0; i < moduleInputs.length; i++) {
            moduleInputs[i] = new ModuleIOInputsAutoLogged();
        }

        final Translation2d[] moduleOffsets = new Translation2d[moduleConfigs.length];
        for (int i = 0; i < moduleOffsets.length; i++) {
            moduleOffsets[i] = moduleConfigs[i].translationOffset();
        }

        this.kinematics = new SwerveDriveKinematics(moduleOffsets);
        this.replayPoseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                Rotation2d.kZero,
                new SwerveModulePosition[] {
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                        new SwerveModulePosition(),
                },
                Pose2d.kZero,
                SwerveConstants.CTRESwerve.OdometryStdDevs,
                SwerveConstants.CTRESwerve.UnusedVisionStdDevs
        );

        this.headingController = new PIDController(4, 0, 0);
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.headingController.setTolerance(Units.degreesToRadians(4), Units.degreesToRadians(6));
        this.atHeadingSetpoint = group.t(
                "atHeadingSetpoint",
                () -> headingControllerActive &&
                        MathUtil.isNear(
                                headingTarget.getRadians(),
                                getPose().getRotation().getRadians(),
                                Units.degreesToRadians(3)
                        ) &&
                        MathUtil.isNear(
                                0,
                                getRobotRelativeSpeeds().omegaRadiansPerSecond,
                                Units.degreesToRadians(12)
                        )
        );

        this.holonomicDriveController = new HolonomicDriveController(
                new PIDController(8, 0, 0.18),
                new PIDController(8, 0, 0.18),
                new PIDController(7, 0, 0.7),
                new TrapezoidProfile.Constraints(
                        Units.feetToMeters(13),
                        Units.feetToMeters(6)
                ),
                new TrapezoidProfile.Constraints(
                        3 * Math.PI,
                        3 * Math.PI
                ),
                this::getFieldRelativeSpeeds,
                new HolonomicDriveController.PositionTolerance(
                        0.025,
                        Rotation2d.fromDegrees(4)
                ),
                new HolonomicDriveController.VelocityTolerance(
                        0.04,
                        Math.PI / 6
                )
        );
        this.atHolonomicDrivePose = holonomicDriveController.atPose(
                this::getPose,
                () -> holonomicPoseTarget
        );
        this.atHolonomicDrivePoseStopped = holonomicDriveController.atPoseAndStopped(
                this::getPose,
                () -> holonomicPoseTarget
        );

        this.holdAxisPID = new PIDController(5, 0, 0.1);

        this.choreoController = new HolonomicChoreoController(
                new PIDController(10, 0, 0.2),
                new PIDController(10, 0, 0.2),
                new PIDController(7, 0, 1.2)
        );

        this.linearVoltageSysIdRoutine = makeLinearVoltageSysIdRoutine();
        this.linearTorqueCurrentSysIdRoutine = makeLinearTorqueCurrentSysIdRoutine();
        this.angularVoltageSysIdRoutine = makeAngularVoltageSysIdRoutine();
    }

    private boolean isReplay() {
        return mode == Constants.RobotMode.REPLAY;
    }

    private double fpgaToCurrentTime(final double fpgaTimeSeconds) {
        return (inputs.currentTimeSeconds - inputs.fpgaTimeSeconds) + fpgaTimeSeconds;
    }

    private double currentTimeToFPGATime(final double currentTimeSeconds) {
        return (inputs.fpgaTimeSeconds - inputs.currentTimeSeconds) + currentTimeSeconds;
    }

    private void updateStateValidCallbacks() {
        final boolean stateValid = inputs.stateValid;
        if (!stateLastValid && stateValid) {
            final SwerveDriveState state = state();
            for (final Consumer<SwerveDriveState> callbacks : onStateValidCallbacks) {
                callbacks.accept(state);
            }

            onStateValidCallbacks.clear();
        }

        stateLastValid = stateValid;
    }

    private double updateOdometry() {
        final double odometryUpdatePeriodSeconds;
        if (isReplay()) {
            final double odometryUpdateStart = Timer.getFPGATimestamp();

            final SwerveDriveState[] states = inputs.states;
            if (!replayPoseEstimatorReset && states.length != 0) {
                final SwerveDriveState oldestState = states[0];
                replayPoseEstimator.resetPosition(oldestState.RawHeading, oldestState.ModulePositions, oldestState.Pose);
                replayPoseEstimatorReset = true;
            }

            double updatePeriodSeconds = 0;
            for (final SwerveDriveState state : states) {
                updatePeriodSeconds = odometryPeriodFilter.calculate(state.OdometryPeriod);
                poseBuffer.addSample(
                        currentTimeToFPGATime(state.Timestamp),
                        replayPoseEstimator.updateWithTime(
                                state.Timestamp,
                                state.RawHeading,
                                state.ModulePositions
                        )
                );
            }

            odometryUpdatePeriodSeconds = updatePeriodSeconds + (Timer.getFPGATimestamp() - odometryUpdateStart);
        } else {
            double updatePeriodSeconds = 0;
            for (final SwerveDriveState state : inputs.states) {
                updatePeriodSeconds = odometryPeriodFilter.calculate(state.OdometryPeriod);
                poseBuffer.addSample(currentTimeToFPGATime(state.Timestamp), state.Pose);
            }

            odometryUpdatePeriodSeconds = updatePeriodSeconds;
        }

        return odometryUpdatePeriodSeconds;
    }

    @Override
    public void periodic() {
        final double swervePeriodicUpdateStart = Timer.getFPGATimestamp();

        swerveIO.updateInputs(inputs, moduleInputs);
        Logger.processInputs(LogKey, inputs);
        for (final ModuleIOInputsAutoLogged moduleInputs : moduleInputs) {
            Logger.processInputs(LogKey + "/Module" + moduleInputs.index, moduleInputs);
        }

        final double odometryUpdatePeriodSeconds = updateOdometry();
        updateStateValidCallbacks();

        if (appliedForwardDirection == null || allowedToChangeForwardDirection.getAsBoolean()) {
            final Rotation2d forwardDirection = Robot.IsRedAlliance.getAsBoolean()
                    ? Rotation2d.k180deg
                    : Rotation2d.kZero;

            if (!forwardDirection.equals(appliedForwardDirection)) {
                swerveIO.setOperatorPerspectiveForward(forwardDirection);
                appliedForwardDirection = forwardDirection;
            }
        }

        final Pose2d robotPose = getPose();
        final ChassisSpeeds robotRelativeSpeeds = getRobotRelativeSpeeds();

        final Transform2d diff = robotPose.minus(state().Pose);
        Logger.recordOutput(LogKey + "/Diff", diff);

        Logger.recordOutput(
                LogKey + "/LinearSpeedMetersPerSecond",
                Math.hypot(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
        );
        Logger.recordOutput(LogKey + "/RobotRelativeChassisSpeeds", robotRelativeSpeeds);
        Logger.recordOutput(LogKey + "/FieldRelativeChassisSpeeds", getFieldRelativeSpeeds());

        Logger.recordOutput(LogKey + "/DesiredStates", getModuleLastDesiredStates());
        Logger.recordOutput(LogKey + "/CurrentStates", getModuleStates());

        Logger.recordOutput(OdometryLogKey + "/Robot2d", robotPose);
        Logger.recordOutput(OdometryLogKey + "/Robot3d", GyroUtils.robotPose2dToPose3dWithGyro(
                robotPose,
                getRotation3d()
        ));

        Logger.recordOutput(LogKey + "/HeadingController/Active", headingControllerActive);
        Logger.recordOutput(LogKey + "/HeadingController/AtHeadingSetpoint", atHeadingSetpoint.getAsBoolean());
        Logger.recordOutput(LogKey + "/HeadingController/TargetHeading", headingTarget);
        Logger.recordOutput(LogKey + "/HeadingController/TargetPose", new Pose2d(
                robotPose.getTranslation(),
                headingTarget
        ));

        Logger.recordOutput(LogKey + "/HolonomicController/Active", holonomicControllerActive);
        Logger.recordOutput(LogKey + "/HolonomicController/TargetPose", holonomicPoseTarget);
        Logger.recordOutput(LogKey + "/HolonomicController/AtHolonomicSetpoint", atHolonomicDrivePose.getAsBoolean());
        Logger.recordOutput(
                LogKey + "/HolonomicController/AtHolonomicSetpointStopped",
                atHolonomicDrivePoseStopped.getAsBoolean()
        );

        Logger.recordOutput(OdometryLogKey + "/OdometryUpdatePeriodSeconds", odometryUpdatePeriodSeconds);
        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                Units.secondsToMilliseconds(Timer.getFPGATimestamp() - swervePeriodicUpdateStart)
        );
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public SwerveDriveState state() {
        if (!inputs.stateValid) {
            DriverStation.reportError("Tried to read invalid SwerveDriveState", true);
        }

        return inputs.state;
    }

    public void onStateValid(final Consumer<SwerveDriveState> callback) {
        if (inputs.stateValid) {
            callback.accept(state());
        } else {
            onStateValidCallbacks.add(callback);
        }
    }

    /**
     * Get the estimated {@link Pose2d} of the robot from the {@link SwerveDrivePoseEstimator}.
     * @return the estimated position of the robot, as a {@link Pose2d}
     */
    public Pose2d getPose() {
        if (isReplay()) {
            return replayPoseEstimator.getEstimatedPosition();
        }
        return state().Pose;
    }

    public Optional<Pose2d> getPose(final double atTimestamp) {
        return poseBuffer.getSample(atTimestamp);
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromRadians(inputs.gyroRotation3d.getY());
    }

    public Rotation2d getRoll() {
        return Rotation2d.fromRadians(inputs.gyroRotation3d.getX());
    }

    public Rotation2d getYaw() {
        return getPose().getRotation();
    }

    public Rotation3d getRotation3d() {
        return new Rotation3d(getRoll().getRadians(), getPitch().getRadians(), getYaw().getRadians());
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return state().Speeds;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getYaw());
    }

    public SwerveModuleState[] getModuleStates() {
        return state().ModuleStates;
    }

    public SwerveModuleState[] getModuleLastDesiredStates() {
        return state().ModuleTargets;
    }

    public SwerveModulePosition[] getModulePositions() {
        return state().ModulePositions;
    }

    public void resetPose(final Pose2d pose) {
        if (isReplay()) {
            replayPoseEstimator.resetPose(pose);
        }
        swerveIO.resetPose(pose);
    }

    public void addVisionMeasurement(
            final Pose2d visionRobotPoseMeters,
            final double fpgaTimestampSeconds,
            final Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        final double currentTime = fpgaToCurrentTime(fpgaTimestampSeconds);
        if (isReplay()) {
            replayPoseEstimator.addVisionMeasurement(
                    visionRobotPoseMeters,
                    currentTime,
                    visionMeasurementStdDevs
            );
        }

        swerveIO.addVisionMeasurement(
                visionRobotPoseMeters,
                currentTime,
                visionMeasurementStdDevs
        );
    }

    private void applyRequest(final SwerveRequest request) {
        swerveIO.setControl(request);
    }

    public void driveFieldRelative(
            final double xSpeedMeterPerSec,
            final double ySpeedMetersPerSec,
            final double omegaRadsPerSec,
            final SwerveRequest.ForwardPerspectiveValue forwardPerspective
    ) {
        applyRequest(driveFieldRelative
                .withVelocityX(xSpeedMeterPerSec)
                .withVelocityY(ySpeedMetersPerSec)
                .withRotationalRate(omegaRadsPerSec)
                .withForwardPerspective(forwardPerspective));
    }

    public void driveRobotRelative(
            final double xSpeedMeterPerSec,
            final double ySpeedMetersPerSec,
            final double omegaRadsPerSec
    ) {
        applyRequest(driveRobotRelative
                .withVelocityX(xSpeedMeterPerSec)
                .withVelocityY(ySpeedMetersPerSec)
                .withRotationalRate(omegaRadsPerSec));
    }

    public Command driveFieldRelative(
            final DoubleSupplier xSpeedMeterPerSec,
            final DoubleSupplier ySpeedMetersPerSec,
            final DoubleSupplier omegaRadsPerSec,
            final SwerveRequest.ForwardPerspectiveValue forwardPerspective
    ) {
        return run(() -> driveFieldRelative(
                xSpeedMeterPerSec.getAsDouble(),
                ySpeedMetersPerSec.getAsDouble(),
                omegaRadsPerSec.getAsDouble(),
                forwardPerspective
        ));
    }

    public Command driveRobotRelative(
            final DoubleSupplier xSpeedMeterPerSec,
            final DoubleSupplier ySpeedMetersPerSec,
            final DoubleSupplier omegaRadsPerSec
    ) {
        return run(() -> driveRobotRelative(
                xSpeedMeterPerSec.getAsDouble(),
                ySpeedMetersPerSec.getAsDouble(),
                omegaRadsPerSec.getAsDouble()
        ));
    }

    public void drive(final ChassisSpeeds speeds) {
        applyRequest(applyRobotSpeeds.withSpeeds(speeds));
    }

    public void drive(final ChassisSpeeds speeds, final double[] moduleForcesX, final double[] moduleForcesY) {
        applyRequest(applyRobotSpeeds
                .withSpeeds(speeds)
                .withWheelForceFeedforwardsX(moduleForcesX)
                .withWheelForceFeedforwardsY(moduleForcesY)
        );
    }

    public Command teleopDriveCommand(
            final DoubleSupplier xSpeedSupplier,
            final DoubleSupplier ySpeedSupplier,
            final DoubleSupplier rotSupplier
    ) {
        return run(() -> {
            final SwerveSpeed.Speeds swerveSpeed = SwerveSpeed.getSwerveSpeed();

            final Translation2d translationInput = ControllerUtils.calculateLinearVelocity(
                    -xSpeedSupplier.getAsDouble(),
                    -ySpeedSupplier.getAsDouble(),
                    0.01
            );

            final double rotationInput = ControllerUtils.getStickSquaredInput(
                    -rotSupplier.getAsDouble(),
                    0.01
            );

            driveFieldRelative(
                    translationInput.getX()
                            * swerveSpeed.getTranslationSpeed(),
                    translationInput.getY()
                            * swerveSpeed.getTranslationSpeed(),
                    rotationInput
                            * swerveSpeed.getRotationSpeed(),
                    SwerveRequest.ForwardPerspectiveValue.OperatorPerspective
            );
        }).withName("TeleopDrive");
    }

    public Command teleopFacingAngle(
            final DoubleSupplier xSpeedSupplier,
            final DoubleSupplier ySpeedSupplier,
            final Supplier<Rotation2d> rotationTargetSupplier
    ) {
        return Commands.sequence(
                        runOnce(() -> {
                            headingControllerActive = true;
                            headingController.reset();
                        }),
                        run(() -> {
                            final SwerveSpeed.Speeds swerveSpeed = SwerveSpeed.getSwerveSpeed();

                            final Translation2d translationInput = ControllerUtils.calculateLinearVelocity(
                                    -xSpeedSupplier.getAsDouble(),
                                    -ySpeedSupplier.getAsDouble(),
                                    0.01
                            );

                            this.headingTarget = rotationTargetSupplier.get();
                            driveFieldRelative(
                                    translationInput.getX()
                                            * swerveSpeed.getTranslationSpeed(),
                                    translationInput.getY()
                                            * swerveSpeed.getTranslationSpeed(),
                                    headingController.calculate(getYaw().getRadians(), headingTarget.getRadians()),
                                    SwerveRequest.ForwardPerspectiveValue.OperatorPerspective
                            );
                        })
                )
                .finallyDo(() -> headingControllerActive = false)
                .withName("TeleopFacingAngle");
    }

    public Command robotRelativeFacingAngle(
            final DoubleSupplier xSpeedSupplier,
            final DoubleSupplier ySpeedSupplier,
            final Supplier<Rotation2d> rotationTargetSupplier
    ) {
        return Commands.sequence(
                        runOnce(() -> {
                            headingControllerActive = true;
                            headingController.reset();
                        }),
                        run(() -> {
                            this.headingTarget = rotationTargetSupplier.get();
                            driveRobotRelative(
                                    xSpeedSupplier.getAsDouble(),
                                    ySpeedSupplier.getAsDouble(),
                                    headingController.calculate(getYaw().getRadians(), headingTarget.getRadians())
                            );
                        })
                )
                .finallyDo(() -> headingControllerActive = false)
                .withName("RobotRelativeFacingAngle");
    }

    public Command faceAngle(final Supplier<Rotation2d> rotationTargetSupplier) {
        return Commands.sequence(
                        runOnce(() -> {
                            headingControllerActive = true;
                            headingController.reset();
                        }),
                        run(() -> {
                            this.headingTarget = rotationTargetSupplier.get();
                            driveFieldRelative(
                                    0,
                                    0,
                                    headingController.calculate(getYaw().getRadians(), headingTarget.getRadians()),
                                    SwerveRequest.ForwardPerspectiveValue.BlueAlliance
                            );
                        })
                )
                .finallyDo(() -> headingControllerActive = false)
                .withName("FaceAngle");
    }

    public Command driveToPose(final Supplier<Pose2d> poseSupplier) {
        return Commands.sequence(
                runOnce(() -> {
                    holonomicControllerActive = true;
                    holonomicPoseTarget = poseSupplier.get();
                    holonomicDriveController.reset(getPose(), holonomicPoseTarget);
                }),
                run(() -> {
                    holonomicPoseTarget = poseSupplier.get();
                    drive(holonomicDriveController.calculate(getPose(), holonomicPoseTarget));
                }).until(atHolonomicDrivePose),
                runOnce(this::stop)
        )
                .finallyDo(() -> holonomicControllerActive = false)
                .withName("DriveToPose");
    }

    public Command runToPose(final Supplier<Pose2d> poseSupplier) {
        return Commands.sequence(
                runOnce(() -> {
                    holonomicControllerActive = true;
                    holonomicPoseTarget = poseSupplier.get();
                    holonomicDriveController.reset(getPose(), holonomicPoseTarget);
                }),
                run(() -> {
                    holonomicPoseTarget = poseSupplier.get();
                    drive(holonomicDriveController.calculate(getPose(), holonomicPoseTarget));
                })
        )
                .finallyDo(() -> holonomicControllerActive = false)
                .withName("RunToPose");
    }

    public Command teleopHoldAxisFacingAngleCommand(
            final double holdPosition,
            final DriveAxis holdAxis,
            final DoubleSupplier speedSupplier,
            final Supplier<Rotation2d> rotationTargetSupplier
    ) {
        return Commands.sequence(
                        runOnce(() -> {
                            headingControllerActive = true;
                            headingController.reset();
                            holdAxisPID.reset();
                        }),
                        run(() -> {
                            final Pose2d currentPose = getPose();
                            this.headingTarget = rotationTargetSupplier.get();

                            final double holdEffort = holdAxisPID.calculate(
                                    holdAxis == DriveAxis.X
                                            ? currentPose.getX()
                                            : currentPose.getY(),
                                    holdPosition
                            );

                            final double xSpeed = holdAxis == DriveAxis.X ? holdEffort : speedSupplier.getAsDouble();
                            final double ySpeed = holdAxis == DriveAxis.Y ? holdEffort : speedSupplier.getAsDouble();
                            driveFieldRelative(
                                    xSpeed,
                                    ySpeed,
                                    headingController.calculate(getYaw().getRadians(), headingTarget.getRadians()),
                                    SwerveRequest.ForwardPerspectiveValue.BlueAlliance
                            );
                        })
                )
                .finallyDo(() -> headingControllerActive = false)
                .withName("TeleopHoldAxisFacingAngle");
    }

    public Command holdAxisFacingAngleAndDrive(
            final DoubleSupplier holdPosition,
            final DriveAxis holdAxis,
            final double driveSpeed,
            final Supplier<Rotation2d> headingTarget
    ) {
        return Commands.sequence(
                        runOnce(() -> {
                            headingControllerActive = true;
                            headingController.reset();
                            holdAxisPID.reset();
                        }),
                        run(() -> {
                            final Pose2d currentPose = getPose();
                            this.headingTarget = headingTarget.get();

                            final double holdEffort = holdAxisPID.calculate(
                                    holdAxis == DriveAxis.X
                                            ? currentPose.getX()
                                            : currentPose.getY(),
                                    holdPosition.getAsDouble()
                            );

                            final double xSpeed = holdAxis == DriveAxis.X ? holdEffort : driveSpeed;
                            final double ySpeed = holdAxis == DriveAxis.Y ? holdEffort : driveSpeed;
                            driveFieldRelative(
                                    xSpeed,
                                    ySpeed,
                                    headingController.calculate(getYaw().getRadians(), this.headingTarget.getRadians()),
                                    SwerveRequest.ForwardPerspectiveValue.BlueAlliance
                            );
                        })
                )
                .finallyDo(() -> headingControllerActive = false)
                .withName("HoldAxisFacingAngleAndDrive");
    }

    public Command driveToAxisFacingAngle(
            final DoubleSupplier holdPosition,
            final DriveAxis holdAxis,
            final Supplier<Rotation2d> headingTarget
    ) {
        final LoggedTrigger atAxis = atAxisTrigger(
                holdPosition,
                holdAxis == DriveAxis.X
                        ? () -> getPose().getX()
                        : () -> getPose().getY()
        );

        return Commands.sequence(
                        runOnce(() -> {
                            headingControllerActive = true;
                            headingController.reset();
                            holdAxisPID.reset();
                        }),
                        run(() -> {
                            final Pose2d currentPose = getPose();
                            this.headingTarget = headingTarget.get();

                            final double holdEffort = holdAxisPID.calculate(
                                    holdAxis == DriveAxis.X
                                            ? currentPose.getX()
                                            : currentPose.getY(),
                                    holdPosition.getAsDouble()
                            );

                            final double xSpeed = holdAxis == DriveAxis.X ? holdEffort : 0;
                            final double ySpeed = holdAxis == DriveAxis.Y ? holdEffort : 0;
                            driveFieldRelative(
                                    xSpeed,
                                    ySpeed,
                                    headingController.calculate(getYaw().getRadians(), this.headingTarget.getRadians()),
                                    SwerveRequest.ForwardPerspectiveValue.BlueAlliance
                            );
                        })
                ).until(atAxis)
                .finallyDo(() -> headingControllerActive = false)
                .withName("HoldAxisFacingAngleAndDrive");
    }

    public void stop() {
        drive(new ChassisSpeeds());
    }

    public Command stopCommand() {
        return runOnce(this::stop)
                .withName("Stop");
    }

    /**
     * Put modules into an X pattern (significantly reduces the swerve's ability to coast/roll)
     */
    public void wheelX() {
        applyRequest(brake);
    }

    public Command wheelXCommand() {
        return runOnce(this::wheelX)
                .withName("WheelX");
    }

    public Command runWheelXCommand() {
        return run(this::wheelX)
                .withName("RunWheelX");
    }

    public LoggedTrigger atPoseTrigger(final Supplier<Pose2d> targetPoseSupplier) {
        return holonomicDriveController.atPose(this::getPose, targetPoseSupplier);
    }

    public LoggedTrigger atPoseTrigger(
            final Supplier<Pose2d> targetPoseSupplier,
            final HolonomicDriveController.PositionTolerance tolerance
    ) {
        return HolonomicDriveController.atPose(group, this::getPose, targetPoseSupplier, tolerance);
    }

    public LoggedTrigger atPoseTrigger(
            final Supplier<Pose2d> targetPoseSupplier,
            final HolonomicDriveController.PositionTolerance positionTolerance,
            final HolonomicDriveController.VelocityTolerance velocityTolerance
    ) {
        return HolonomicDriveController.atPoseAndStopped(
                group,
                this::getPose,
                this::getFieldRelativeSpeeds,
                targetPoseSupplier,
                positionTolerance,
                velocityTolerance);
    }

    public LoggedTrigger atPoseAndStoppedTrigger(final Supplier<Pose2d> targetPoseSupplier) {
        return holonomicDriveController.atPoseAndStopped(
                this::getPose,
                targetPoseSupplier
        );
    }

    public LoggedTrigger atAxisTrigger(final DoubleSupplier target, final DoubleSupplier measurement) {
        final DoubleSupplier linearSpeedSupplier = () -> {
            final ChassisSpeeds speeds = getFieldRelativeSpeeds();
            return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        };

        return group.t(
                "atAxis",
                () -> MathUtil.isNear(
                        target.getAsDouble(),
                        measurement.getAsDouble(),
                        0.05
                ) && MathUtil.isNear(
                        0,
                        linearSpeedSupplier.getAsDouble(),
                        0.05
                )
        );
    }

//    public void followChoreoSample(final SwerveSample swerveSample) {
//        final Pose2d currentPose = getPose();
//        final ChassisSpeeds speeds = choreoController.calculate(currentPose, swerveSample);
//
//        Logger.recordOutput(Autos.LogKey + "/Timestamp", swerveSample.getTimestamp());
//        Logger.recordOutput(Autos.LogKey + "/CurrentPose", currentPose);
//        Logger.recordOutput(Autos.LogKey + "/TargetSpeeds", swerveSample.getChassisSpeeds());
//        Logger.recordOutput(Autos.LogKey + "/TargetPose", swerveSample.getPose());
//
//        Logger.recordOutput(
//                Autos.LogKey + "/TargetRotation",
//                MathUtil.angleModulus(swerveSample.heading)
//        );
//
//        Logger.recordOutput(
//                Autos.LogKey + "/CurrentRotation",
//                MathUtil.angleModulus(currentPose.getRotation().getRadians())
//        );
//
////        drive(speeds, swerveSample.moduleForcesX(), swerveSample.moduleForcesY());
//        drive(speeds);
//    }

    @SuppressWarnings("unused")
    public Command wheelRadiusCharacterization() {
        final double wheelRadiusMaxVelocityRadsPerSec = 0.25;
        final double wheelRadiusRampRateRadPerSecSquared = 0.05;

        final SlewRateLimiter limiter = new SlewRateLimiter(wheelRadiusRampRateRadPerSecSquared);
        final WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        final Supplier<double[]> drivePositionsSupplier = () -> {
            final SwerveModulePosition[] positions = getModulePositions();
            final double[] positionMeters = new double[positions.length];
            for (int i = 0; i < positionMeters.length; i++) {
                positionMeters[i] = positions[i].distanceMeters;
            }

            return positionMeters;
        };

        final Supplier<Rotation2d> yawRotation2dSupplier = this::getYaw;

        return Commands.parallel(
                Commands.sequence(
                        Commands.runOnce(() -> limiter.reset(0.0)),
                        run(() -> {
                            final double speed = limiter.calculate(wheelRadiusMaxVelocityRadsPerSec);
                            drive(new ChassisSpeeds(0.0, 0.0, speed));
                        })
                ),
                Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(() -> {
                            state.positions = drivePositionsSupplier.get();
                            state.gyroDelta = 0.0;
                            state.lastAngle = yawRotation2dSupplier.get();
                        }),
                        Commands.run(() -> {
                            final Rotation2d rotation = yawRotation2dSupplier.get();
                            state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRotations());
                            state.lastAngle = rotation;
                        }).finallyDo(() -> {
                            final double[] positions = drivePositionsSupplier.get();
                            double wheelDeltaRots = 0.0;
                            for (int i = 0; i < 4; i++) {
                                wheelDeltaRots += Math.abs(positions[i] - state.positions[i]) / 4.0;
                            }

                            final double wheelRadius =
                                    (state.gyroDelta * Config.driveBaseRadiusMeters()) / wheelDeltaRots;

                            final NumberFormat formatter = new DecimalFormat("#0.000000000000000000000000000");
                            System.out.println("********** Wheel Radius Characterization Results **********");
                            System.out.println("\tWheel Delta: " + formatter.format(wheelDeltaRots) + " rotations");
                            System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " rotations");
                            System.out.println("\tWheel Radius: " + formatter.format(wheelRadius) + " meters");

                            stop();
                        })
                )
        );
    }

    public static class RequestSysIdSwerveTranslationTorqueCurrent implements SwerveRequest {
        public double torqueCurrentToApply = 0;

        private final TorqueCurrentFOC driveRequest = new TorqueCurrentFOC(0);
        private final PositionVoltage steerRequestVoltage = new PositionVoltage(0);
        private final PositionTorqueCurrentFOC steerRequestTorqueCurrent = new PositionTorqueCurrentFOC(0);

        public StatusCode apply(
                final SwerveDrivetrain.SwerveControlParameters parameters,
                final SwerveModule<?, ?, ?>... modulesToApply)
        {
            for (final SwerveModule<?, ?, ?> swerveModule : modulesToApply) {
                switch (swerveModule.getSteerClosedLoopOutputType()) {
                    case Voltage:
                        swerveModule.apply(
                                driveRequest.withOutput(torqueCurrentToApply),
                                steerRequestVoltage.withPosition(0)
                        );
                        break;
                    case TorqueCurrentFOC:
                        swerveModule.apply(
                                driveRequest.withOutput(torqueCurrentToApply),
                                steerRequestTorqueCurrent.withPosition(0)
                        );
                        break;
                }
            }
            return StatusCode.OK;
        }

        public RequestSysIdSwerveTranslationTorqueCurrent withTorqueCurrent(final double torqueCurrentAmps) {
            torqueCurrentToApply = torqueCurrentAmps;
            return this;
        }

        public RequestSysIdSwerveTranslationTorqueCurrent withTorqueCurrent(final Current torqueCurrent) {
            torqueCurrentToApply = torqueCurrent.in(Amps);
            return this;
        }
    }

    public static class RequestSysIdSwerveRotationVoltage implements SwerveRequest {
        public double voltsToApply = 0;

        private final VoltageOut driveRequest = new VoltageOut(0);
        private final PositionVoltage steerRequestVoltage = new PositionVoltage(0);
        private final PositionTorqueCurrentFOC steerRequestTorqueCurrent = new PositionTorqueCurrentFOC(0);

        public StatusCode apply(
                final SwerveDrivetrain.SwerveControlParameters parameters,
                final SwerveModule<?, ?, ?>... modulesToApply)
        {
            for (int i = 0; i < modulesToApply.length; ++i) {
                final var angle = parameters.moduleLocations[i].getAngle().plus(Rotation2d.kCCW_90deg);
                final var swerveModule = modulesToApply[i];

                switch (swerveModule.getSteerClosedLoopOutputType()) {
                    case Voltage:
                        swerveModule.apply(
                                driveRequest.withOutput(voltsToApply),
                                steerRequestVoltage.withPosition(angle.getRotations())
                        );
                        break;
                    case TorqueCurrentFOC:
                        swerveModule.apply(
                                driveRequest.withOutput(voltsToApply),
                                steerRequestTorqueCurrent.withPosition(angle.getRotations())
                        );
                        break;
                }
            }
            return StatusCode.OK;
        }

        public RequestSysIdSwerveRotationVoltage withVolts(final double volts) {
            voltsToApply = volts;
            return this;
        }

        public RequestSysIdSwerveRotationVoltage withVolts(final Voltage volts) {
            voltsToApply = volts.in(Volts);
            return this;
        }
    }

    private SysIdRoutine makeLinearVoltageSysIdRoutine() {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(2).per(Second),
                        Volts.of(6),
                        Seconds.of(12),
                        state -> SignalLogger.writeString(LogKey + "-state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        volts -> applyRequest(sysIdTranslationVoltage.withVolts(volts)),
                        null,
                        this
                )
        );
    }

    @SuppressWarnings("unused")
    public Command linearVoltageSysIdQuasistaticCommand(final SysIdRoutine.Direction direction) {
        return linearVoltageSysIdRoutine.quasistatic(direction);
    }

    @SuppressWarnings("unused")
    public Command linearVoltageSysIdDynamicCommand(final SysIdRoutine.Direction direction) {
        return linearVoltageSysIdRoutine.dynamic(direction);
    }

    private SysIdRoutine makeLinearTorqueCurrentSysIdRoutine() {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        // this is actually amps/sec not volts/sec
                        Volts.of(4).per(Second),
                        // this is actually amps not volts
                        Volts.of(12),
                        Seconds.of(20),
                        state -> SignalLogger.writeString(LogKey + "-state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> {
//                             convert the voltage measure to an amperage measure by tricking it
                            final Current torqueCurrent = Amps.of(voltageMeasure.magnitude());
                            applyRequest(sysIdTranslationTorqueCurrent.withTorqueCurrent(torqueCurrent));
                        },
                        null,
                        this
                )
        );
    }

    @SuppressWarnings("unused")
    public Command linearTorqueCurrentSysIdQuasistaticCommand(final SysIdRoutine.Direction direction) {
        return linearTorqueCurrentSysIdRoutine.quasistatic(direction);
    }

    @SuppressWarnings("unused")
    public Command linearTorqueCurrentSysIdDynamicCommand(final SysIdRoutine.Direction direction) {
        return linearTorqueCurrentSysIdRoutine.dynamic(direction);
    }

    private SysIdRoutine makeAngularVoltageSysIdRoutine() {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        // this is actually amps/sec not volts/sec
                        Volts.of(1).per(Second),
                        Volts.of(10),
                        Seconds.of(20),
                        state -> SignalLogger.writeString("state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        volts -> applyRequest(sysIdSwerveRotationVoltage.withVolts(volts)),
                        null,
                        this
                )
        );
    }

    @SuppressWarnings("unused")
    public Command angularVoltageSysIdQuasistaticCommand(final SysIdRoutine.Direction direction) {
        return angularVoltageSysIdRoutine.quasistatic(direction);
    }

    @SuppressWarnings("unused")
    public Command angularVoltageSysIdDynamicCommand(final SysIdRoutine.Direction direction) {
        return angularVoltageSysIdRoutine.dynamic(direction);
    }
}
