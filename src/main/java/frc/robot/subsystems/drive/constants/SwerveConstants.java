package frc.robot.subsystems.drive.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;

public class SwerveConstants {
    private static final DCMotor KrakenX60Foc = DCMotor.getKrakenX60Foc(1);

    public static final int ModuleCount = 4;
    public static final SwerveConfig Config = new SwerveConfig(
            HardwareConstants.CANBus.CANIVORE,
            Units.inchesToMeters(2),
            6.03,
            287.0 / 11.0,
            3.375,
            Units.inchesToMeters(22.75),
            Units.inchesToMeters(22.75),
            Units.feetToMeters(15.0),
            4 * Math.PI,
            6 * Math.PI,
            Translation2d.kZero
    );

    public record GyroConstants(
            HardwareConstants.CANBus CANBus,
            int gyroId
    ) {}

    public static final GyroConstants Gyro = new GyroConstants(
            Config.canBus,
            13
    );

    public static final SwerveModuleConfig FrontLeftModule = new SwerveModuleConfig(
            "FrontLeft",
            Config.canBus,
            new Translation2d(Config.wheelBaseMeters / 2, Config.trackWidthMeters / 2),
            1,
            2,
            3,
            -0.074,
            SwerveConstants.KrakenX60Foc.KtNMPerAmp
    );

    public static final SwerveModuleConfig FrontRightModule = new SwerveModuleConfig(
            "FrontRight",
            Config.canBus,
            new Translation2d(Config.wheelBaseMeters / 2, -Config.trackWidthMeters / 2),
            4,
            5,
            6,
            -0.662,
            SwerveConstants.KrakenX60Foc.KtNMPerAmp
    );

    public static final SwerveModuleConfig BackLeftModule = new SwerveModuleConfig(
            "BackLeft",
            Config.canBus,
            new Translation2d(-Config.wheelBaseMeters / 2, Config.trackWidthMeters / 2),
            7,
            8,
            9,
            -0.064,
            SwerveConstants.KrakenX60Foc.KtNMPerAmp
    );

    public static final SwerveModuleConfig BackRightModule = new SwerveModuleConfig(
            "BackRight",
            Config.canBus,
            new Translation2d(-Config.wheelBaseMeters / 2, -Config.trackWidthMeters / 2),
            10,
            11,
            12,
            -0.319,
            SwerveConstants.KrakenX60Foc.KtNMPerAmp
    );

    public static class CTRESwerve {
        public static final double OdometryFreqHz = 250;
        public static final Vector<N3> OdometryStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(1));
        public static final Vector<N3> UnusedVisionStdDevs = VecBuilder.fill(0.6, 0.6, Units.degreesToRadians(80));
        public static final int BufferSize = 40;
        static {
            final int minBufferSize = (int) Math.ceil(Constants.LOOP_PERIOD_SECONDS * OdometryFreqHz);
            if (BufferSize < minBufferSize) {
                throw new RuntimeException(String.format("OdometryBufferSize of %d is too small, expected size >= %d," +
                        "since it must fit at least all states recorded within a loop cycle!",
                        BufferSize, minBufferSize));
            }
        }

        private static final Slot0Configs DriveGains = new Slot0Configs()
                .withKS(1).withKV(0.1).withKA(2)
                .withKP(5).withKD(0);

        private static final Slot0Configs SteerGains = new Slot0Configs()
                .withKS(0).withKV(0).withKA(0)
                .withKP(50).withKD(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        private static final Pigeon2Configuration Pigeon2Configuration = new Pigeon2Configuration(); static {
            Pigeon2Configuration.MountPose.MountPoseRoll = -0.7136042714118958;
            Pigeon2Configuration.MountPose.MountPosePitch = 0.5430453419685364;
            Pigeon2Configuration.MountPose.MountPoseYaw = -89.58695220947266;
        }

        private static final double SlipCurrentAmps = 70;
        private static final InvertedValue DriveMotorInverted = InvertedValue.Clockwise_Positive;
        private static final InvertedValue TurnMotorInverted = InvertedValue.CounterClockwise_Positive;
        private static final SensorDirectionValue TurnEncoderDirection = SensorDirectionValue.CounterClockwise_Positive;

        private static final TalonFXConfiguration DriveTalonFXConfiguration = new TalonFXConfiguration(); static {
            DriveTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = SlipCurrentAmps;
            DriveTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -SlipCurrentAmps;
        }

        private static final TalonFXConfiguration TurnTalonFXConfiguration = new TalonFXConfiguration(); static {
            TurnTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 40;
            TurnTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
            TurnTalonFXConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
            TurnTalonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        }

        private static final CANcoderConfiguration CanCoderConfiguration = new CANcoderConfiguration(); static {
            CanCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        }

        private static final double DriveInertiaKgMSquared = 0.01;
        private static final double SteerInertiaKgMSquared = 0.01;

        private static final double DriveKsFrictionVolts = 0.2;
        private static final double TurnKsFrictionVolts = 0.2;

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(Config.canBus.name())
                .withPigeon2Id(Gyro.gyroId)
                .withPigeon2Configs(Pigeon2Configuration);

        public static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
                new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                        .withDriveMotorGearRatio(Config.driveReduction)
                        .withSteerMotorGearRatio(Config.turnReduction)
                        .withCouplingGearRatio(Config.couplingRatio)
                        .withWheelRadius(Config.wheelRadiusMeters)
                        .withDriveMotorGains(DriveGains)
                        .withSteerMotorGains(SteerGains)
                        .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.TorqueCurrentFOC)
                        .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                        .withSlipCurrent(SlipCurrentAmps)
                        .withSpeedAt12Volts(Config.maxLinearVelocityMeterPerSec)
                        .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                        .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                        .withDriveMotorInitialConfigs(DriveTalonFXConfiguration)
                        .withSteerMotorInitialConfigs(TurnTalonFXConfiguration)
                        .withEncoderInitialConfigs(CanCoderConfiguration)
                        .withDriveInertia(DriveInertiaKgMSquared)
                        .withSteerInertia(SteerInertiaKgMSquared)
                        .withDriveFrictionVoltage(DriveKsFrictionVolts)
                        .withSteerFrictionVoltage(TurnKsFrictionVolts);

        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                FrontLeft = createModuleConstants(FrontLeftModule);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                FrontRight = createModuleConstants(FrontRightModule);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                BackLeft = createModuleConstants(BackLeftModule);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                BackRight = createModuleConstants(BackRightModule);

        private static boolean isInverted(final InvertedValue invertedValue) {
            return switch (invertedValue) {
                case CounterClockwise_Positive -> false;
                case Clockwise_Positive -> true;
            };
        }

        @SuppressWarnings("SameParameterValue")
        private static boolean isInverted(final SensorDirectionValue sensorDirectionValue) {
            return switch (sensorDirectionValue) {
                case CounterClockwise_Positive -> false;
                case Clockwise_Positive -> true;
            };
        }

        private static SwerveModuleConstants<
                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        createModuleConstants(final SwerveModuleConfig config) {
            return ConstantCreator.createModuleConstants(
                    config.turnMotorId,
                    config.driveMotorId,
                    config.turnEncoderId,
                    config.turnEncoderOffsetRots,
                    config.translationOffset.getX(),
                    config.translationOffset.getY(),
                    isInverted(DriveMotorInverted),
                    isInverted(TurnMotorInverted),
                    isInverted(TurnEncoderDirection)
            );
        }
    }

    public record SwerveConfig(
            HardwareConstants.CANBus canBus,
            double wheelRadiusMeters,
            double driveReduction,
            double turnReduction,
            double couplingRatio,
            double wheelBaseMeters,
            double trackWidthMeters,
            double maxLinearVelocityMeterPerSec,
            double maxAngularVelocityRadsPerSec,
            double maxAngularAccelerationRadsPerSecSquared,
            Translation2d centerOfRotationMeters
    ) {
        public double driveBaseRadiusMeters() {
            return Math.hypot(wheelBaseMeters / 2, trackWidthMeters / 2);
        }

        public double wheelCircumferenceMeters() {
            return 2 * Math.PI * wheelRadiusMeters;
        }
    }

    public record SwerveModuleConfig(
            String name,
            HardwareConstants.CANBus moduleCANBus,
            Translation2d translationOffset,
            int driveMotorId,
            int turnMotorId,
            int turnEncoderId,
            double turnEncoderOffsetRots,
            double driveMotorKtNmPerAmp
    ) {
    }
}
