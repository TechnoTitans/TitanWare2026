package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.constants.SwerveConstants.CTRESwerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class SwerveIOReal implements SwerveIO {
    private final Lock bufferLock;
    private int bufferMaxSize = 0;
    private int bufferOverflowCount = 0;
    private final int bufferCapacity = CTRESwerve.BufferSize;
    private CircularBuffer<SwerveDrivetrain.SwerveDriveState> stateBuffer;
    private CircularBuffer<SwerveDrivetrain.SwerveDriveState> tmpStateBuffer;

    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    private final ActuallyUsableModule[] modules;

    @SafeVarargs
    public SwerveIOReal(
            final SwerveDrivetrainConstants drivetrainConstants,
            final SwerveModuleConstants<
                    TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... moduleConstants
    ) {
        this.bufferLock = new ReentrantLock();
        this.stateBuffer = new CircularBuffer<>(bufferCapacity);
        this.tmpStateBuffer = new CircularBuffer<>(bufferCapacity);
        this.drivetrain = new SwerveDrivetrain<>(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, CTRESwerve.OdometryFreqHz,
                CTRESwerve.OdometryStdDevs,
                CTRESwerve.UnusedVisionStdDevs,
                moduleConstants
        );

        final SwerveModule<TalonFX, TalonFX, CANcoder>[] swerveModules = drivetrain.getModules();
        final ActuallyUsableModule[] modules = new ActuallyUsableModule[swerveModules.length];
        for (int i = 0; i < modules.length; i++) {
            modules[i] = ActuallyUsableModule.fromSwerveModule(i, swerveModules[i]);
        }
        this.modules = modules;

        this.drivetrain.registerTelemetry(state -> {
            try {
                bufferLock.lock();
                stateBuffer.addLast(state.clone());

                final int size = stateBuffer.size();
                bufferMaxSize = size;
                if (size >= bufferCapacity) {
                    bufferOverflowCount++;
                }
            } finally {
                bufferLock.unlock();
            }
        });
    }

    @Override
    public void updateInputs(final SwerveIOInputs inputs, final ModuleIOInputs[] moduleIOInputs) {
        final int maxSize;
        final int overflowCount;
        final CircularBuffer<SwerveDrivetrain.SwerveDriveState> freeBuffer = stateBuffer;
        try {
            bufferLock.lock();

            maxSize = bufferMaxSize;
            bufferMaxSize = 0;
            overflowCount = bufferOverflowCount;
            bufferOverflowCount = 0;

            stateBuffer = tmpStateBuffer;
            tmpStateBuffer = freeBuffer;
        } finally {
            bufferLock.unlock();
        }

        final int nStates = freeBuffer.size();
        final SwerveDriveState[] states = new SwerveDriveState[nStates];
        for (int i = 0; i < nStates; i++) {
            states[i] = new SwerveDriveState(freeBuffer.removeFirst());
        }

        final boolean hasValidState = nStates > 0;
        inputs.bufferMaxSize = maxSize;
        inputs.bufferOverflowCount = overflowCount;
        inputs.stateValid = hasValidState;
        if (hasValidState) {
            inputs.state = states[nStates - 1];
        }
        inputs.states = states;
        inputs.gyroRotation3d = drivetrain.getRotation3d();
        inputs.fpgaTimeSeconds = Timer.getFPGATimestamp();
        inputs.currentTimeSeconds = Utils.getCurrentTimeSeconds();

        for (int i = 0; i < modules.length; i++) {
            modules[i].updateInputs(moduleIOInputs[i]);
        }
    }

    @Override
    public void setControl(final SwerveRequest request) {
        drivetrain.setControl(request);
    }

    @Override
    public void resetPose(final Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    @Override
    public void addVisionMeasurement(
            final Pose2d visionRobotPoseMeters,
            final double currentTimestampSeconds,
            final Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        drivetrain.addVisionMeasurement(
                visionRobotPoseMeters,
                currentTimestampSeconds,
                visionMeasurementStdDevs
        );
    }

    @Override
    public void setOperatorPerspectiveForward(final Rotation2d forwardDirection) {
        drivetrain.setOperatorPerspectiveForward(forwardDirection);
    }
}
