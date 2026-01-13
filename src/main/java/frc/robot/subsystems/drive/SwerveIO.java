package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.constants.HardwareConstants.CANBus;
import frc.robot.utils.ctre.RefreshAll;
import org.littletonrobotics.junction.AutoLog;

import java.nio.ByteBuffer;

import static frc.robot.subsystems.drive.constants.SwerveConstants.ModuleCount;

public interface SwerveIO {
    @AutoLog
    class SwerveIOInputs {
        public boolean stateValid = false;
        public int bufferMaxSize = 0;
        public int bufferOverflowCount = 0;
        public SwerveDriveState state = SwerveDriveState.EmptyState;
        public SwerveDriveState[] states = new SwerveDriveState[0];

        public Rotation3d gyroRotation3d = Rotation3d.kZero;
        public double fpgaTimeSeconds = 0;
        public double currentTimeSeconds = 0;

//        public double[] fpgaTimestamps = new double[0];
    }

    @AutoLog
    class ModuleIOInputs {
        public int index = 0;

        public double drivePositionRots = 0;
        public double driveVelocityRotsPerSec = 0;
        public double driveTorqueCurrentAmps = 0;
        public double driveTempCelsius = 0;

        public double turnPositionRots = 0;
        public double turnVelocityRotsPerSec = 0;
        public double turnTorqueCurrentAmps = 0;
        public double turnTempCelsius = 0;
    }

    class Module<
            DriveMotorT extends CommonTalon,
            TurnMotorT extends CommonTalon,
            EncoderT extends ParentDevice> {
        public final int index;
        public final DriveMotorT driveMotor;
        public final TurnMotorT turnMotor;
        public final EncoderT encoder;

        private final StatusSignal<Angle> drivePosition;
        private final StatusSignal<AngularVelocity> driveVelocity;
        private final StatusSignal<Current> driveTorqueCurrent;
        private final StatusSignal<Temperature> driveTemperature;

        private final StatusSignal<Angle> turnPosition;
        private final StatusSignal<AngularVelocity> turnVelocity;
        private final StatusSignal<Current> turnTorqueCurrent;
        private final StatusSignal<Temperature> turnTemperature;

        private Module(
                final int index,
                final DriveMotorT driveMotor,
                final TurnMotorT turnMotor,
                final EncoderT encoder
        ) {
            this.index = index;
            this.driveMotor = driveMotor;
            this.turnMotor = turnMotor;
            this.encoder = encoder;

            this.drivePosition = driveMotor.getPosition(false);
            this.driveVelocity = driveMotor.getVelocity(false);
            this.driveTorqueCurrent = driveMotor.getTorqueCurrent(false);
            this.driveTemperature = driveMotor.getDeviceTemp(false);

            this.turnPosition = turnMotor.getPosition(false);
            this.turnVelocity = turnMotor.getVelocity(false);
            this.turnTorqueCurrent = turnMotor.getTorqueCurrent(false);
            this.turnTemperature = turnMotor.getDeviceTemp(false);

            RefreshAll.add(CANBus.fromPhoenix6CANBus(driveMotor.getNetwork()),
                    drivePosition, driveVelocity, turnTorqueCurrent, driveTemperature);
            RefreshAll.add(CANBus.fromPhoenix6CANBus(turnMotor.getNetwork()),
                    turnPosition, turnVelocity, turnTorqueCurrent, turnTemperature);
        }

        public void updateInputs(final ModuleIOInputs inputs) {
            inputs.index = index;

            inputs.drivePositionRots = drivePosition.getValueAsDouble();
            inputs.driveVelocityRotsPerSec = driveVelocity.getValueAsDouble();
            inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();
            inputs.driveTempCelsius = driveTemperature.getValueAsDouble();

            inputs.turnPositionRots = turnPosition.getValueAsDouble();
            inputs.turnVelocityRotsPerSec = turnVelocity.getValueAsDouble();
            inputs.turnTorqueCurrentAmps = turnTorqueCurrent.getValueAsDouble();
            inputs.turnTempCelsius = turnTemperature.getValueAsDouble();
        }
    }

    class ActuallyUsableModule extends Module<TalonFX, TalonFX, CANcoder> {
        private ActuallyUsableModule(final int index,
                                     final SwerveModule<TalonFX, TalonFX, CANcoder> module) {
            super(index, module.getDriveMotor(), module.getSteerMotor(), module.getEncoder());
        }

        public static ActuallyUsableModule fromSwerveModule(
                final int index,
                final SwerveModule<TalonFX, TalonFX, CANcoder> module
        ) {
            return new ActuallyUsableModule(index, module);
        }
    }

    default void updateInputs(final SwerveIOInputs inputs, final ModuleIOInputs[] moduleIOInputs) {}

    default void setControl(final SwerveRequest request) {}

    default void resetPose(final Pose2d pose) {}

    default void addVisionMeasurement(
            final Pose2d visionRobotPoseMeters,
            final double currentTimestampSeconds,
            final Matrix<N3, N1> visionMeasurementStdDevs
    ) {}

    default void setOperatorPerspectiveForward(final Rotation2d forwardDirection) {}

    class SwerveDriveState extends SwerveDrivetrain.SwerveDriveState implements StructSerializable {
        @SuppressWarnings("unused")
        public static final SwerveDriveStateStruct struct = new SwerveDriveStateStruct();
        public static final SwerveDriveState EmptyState = new SwerveDriveState();
        static {
            EmptyState.ModuleStates = new SwerveModuleState[ModuleCount];
            EmptyState.ModuleTargets = new SwerveModuleState[ModuleCount];
            EmptyState.ModulePositions = new SwerveModulePosition[ModuleCount];
            for (int i = 0; i < ModuleCount; i++) {
                EmptyState.ModuleStates[i] = new SwerveModuleState();
                EmptyState.ModuleTargets[i] = new SwerveModuleState();
                EmptyState.ModulePositions[i] = new SwerveModulePosition();
            }
        }

        /**
         * Call {@link Swerve#getPose()} instead.
         * Directly accessing this {@link Pose2d} is nondeterministic in replay.
         */
        protected Pose2d Pose;

        private SwerveDriveState() {}

        public SwerveDriveState(final SwerveDrivetrain.SwerveDriveState state) {
            this.Pose = state.Pose;
            this.Speeds = state.Speeds;
            this.ModuleStates = state.ModuleStates;
            this.ModuleTargets = state.ModuleTargets;
            this.ModulePositions = state.ModulePositions;
            this.RawHeading = state.RawHeading;
            this.Timestamp = state.Timestamp;
            this.OdometryPeriod = state.OdometryPeriod;
            this.SuccessfulDaqs = state.SuccessfulDaqs;
            this.FailedDaqs = state.FailedDaqs;
        }
    }

    class SwerveDriveStateStruct implements Struct<SwerveDriveState> {
        @Override
        public Class<SwerveDriveState> getTypeClass() {
            return SwerveDriveState.class;
        }

        @Override
        public String getTypeName() {
            return "Swerve.SwerveDriveState";
        }

        @Override
        public int getSize() {
            return Pose2d.struct.getSize() // Pose
                    + ChassisSpeeds.struct.getSize() // Speeds
                    + (ModuleCount * SwerveModuleState.struct.getSize()) // ModuleStates[4]
                    + (ModuleCount * SwerveModuleState.struct.getSize()) // ModuleTargets[4]
                    + (ModuleCount * SwerveModulePosition.struct.getSize()) // ModulePositions[4]
                    + Rotation2d.struct.getSize() // RawHeading
                    + kSizeDouble // Timestamp
                    + kSizeDouble // OdometryPeriod
                    + kSizeInt32 // SuccessfulDaqs
                    + kSizeInt32; // FailedDaqs
        }

        @Override
        public String getSchema() {
            return "Pose2d Pose; ChassisSpeeds Speeds; SwerveModuleState ModuleStates[4];"
                + "SwerveModuleState ModuleTargets[4]; SwerveModulePosition ModulePositions[4];"
                + "Rotation2d RawHeading; double Timestamp; double OdometryPeriod; int32 SuccessfulDaqs;"
                + "int32 FailedDaqs";
        }

        @Override
        public Struct<?>[] getNested() {
            return new Struct<?>[] {
                    Pose2d.struct,
                    ChassisSpeeds.struct,
                    SwerveModuleState.struct,
                    SwerveModulePosition.struct,
                    Rotation2d.struct
            };
        }

        @Override
        public SwerveDriveState unpack(final ByteBuffer bb) {
            final SwerveDriveState state = new SwerveDriveState();
            state.Pose = Pose2d.struct.unpack(bb);
            state.Speeds = ChassisSpeeds.struct.unpack(bb);

            state.ModuleStates = new SwerveModuleState[ModuleCount];
            for (int i = 0; i < ModuleCount; i++) {
                state.ModuleStates[i] = SwerveModuleState.struct.unpack(bb);
            }

            state.ModuleTargets = new SwerveModuleState[ModuleCount];
            for (int i = 0; i < ModuleCount; i++) {
                state.ModuleTargets[i] = SwerveModuleState.struct.unpack(bb);
            }

            state.ModulePositions = new SwerveModulePosition[ModuleCount];
            for (int i = 0; i < ModuleCount; i++) {
                state.ModulePositions[i] = SwerveModulePosition.struct.unpack(bb);
            }

            state.RawHeading = Rotation2d.struct.unpack(bb);
            state.Timestamp = bb.getDouble();
            state.OdometryPeriod = bb.getDouble();
            state.SuccessfulDaqs = bb.getInt();
            state.FailedDaqs = bb.getInt();

            return state;
        }

        @Override
        public void pack(final ByteBuffer bb, final SwerveDriveState state) {
            Pose2d.struct.pack(bb, state.Pose);
            ChassisSpeeds.struct.pack(bb, state.Speeds);

            for (int i = 0; i < ModuleCount; i++) {
                SwerveModuleState.struct.pack(bb, state.ModuleStates[i]);
            }

            for (int i = 0; i < ModuleCount; i++) {
                SwerveModuleState.struct.pack(bb, state.ModuleTargets[i]);
            }

            for (int i = 0; i < ModuleCount; i++) {
                SwerveModulePosition.struct.pack(bb, state.ModulePositions[i]);
            }

            Rotation2d.struct.pack(bb, state.RawHeading);
            bb.putDouble(state.Timestamp);
            bb.putDouble(state.OdometryPeriod);
            bb.putInt(state.SuccessfulDaqs);
            bb.putInt(state.FailedDaqs);
        }
    }
}
