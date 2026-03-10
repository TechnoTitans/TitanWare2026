package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;

public class TurretIOReal implements TurretIO {
    private final HardwareConstants.TurretConstants constants;

    private final TalonFX motor;
    private final CANcoder primaryCANcoder;
    private final CANcoder secondaryCANcoder;

    private final PositionVoltage positionVoltage;
    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorDeviceTemp;

    private final StatusSignal<Angle> primaryCANcoderPosition;
    private final StatusSignal<Angle> secondaryCANcoderPosition;

    public TurretIOReal(final HardwareConstants.TurretConstants constants) {
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        final CANBus p6Bus = bus.toPhoenix6CANBus();
        this.motor = new TalonFX(constants.motorId(), p6Bus);
        this.primaryCANcoder = new CANcoder(constants.primaryCANcoderId(), p6Bus);
        this.secondaryCANcoder = new CANcoder(constants.secondaryCANcoderId(), p6Bus);

        this.positionVoltage = new PositionVoltage(0);
        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.voltageOut = new VoltageOut(0);

        this.motorPosition = motor.getPosition(false);
        this.motorVelocity = motor.getVelocity(false);
        this.motorVoltage = motor.getMotorVoltage(false);
        this.motorTorqueCurrent = motor.getTorqueCurrent(false);
        this.motorDeviceTemp = motor.getDeviceTemp(false);

        this.primaryCANcoderPosition = primaryCANcoder.getPosition(false);
        this.secondaryCANcoderPosition = secondaryCANcoder.getPosition(false);

        RefreshAll.add(
                bus,
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                motorDeviceTemp,
                primaryCANcoderPosition,
                secondaryCANcoderPosition
        );
    }

    @Override
    public void updateInputs(final TurretIOInputs inputs) {
        inputs.motorPositionRots = motorPosition.getValueAsDouble();
        inputs.motorVelocityRotsPerSec = motorVelocity.getValueAsDouble();
        inputs.motorVoltage = motorVoltage.getValueAsDouble();
        inputs.motorTorqueCurrentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.motorTempCelsius = motorDeviceTemp.getValueAsDouble();

        inputs.primaryCANcoderPositionRots = primaryCANcoderPosition.getValueAsDouble();
        inputs.secondaryCANcoderPositionRots = secondaryCANcoderPosition.getValueAsDouble();
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0 = new Slot0Configs()
                .withKS(0.366)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKV(5)
                .withKP(30)
                .withKD(0);
        motorConfiguration.Slot1 = new Slot1Configs()
                .withKS(0.366)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKP(80)
                .withKD(0.45);
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        motorConfiguration.MotionMagic.MotionMagicExpo_kV = 0.12;
        motorConfiguration.MotionMagic.MotionMagicExpo_kA = 0.1;
        motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfiguration.Feedback.RotorToSensorRatio = 1;
        motorConfiguration.Feedback.SensorToMechanismRatio = constants.motorToGearboxGearing()
                * constants.gearboxToTurretGearing();
        motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.forwardLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.reverseLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfiguration));

        final CANcoderConfiguration primaryCANcoderConfiguration = new CANcoderConfiguration();
        primaryCANcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        primaryCANcoderConfiguration.MagnetSensor.MagnetOffset = constants.primaryCANcoderOffsetRots();
        primaryCANcoder.getConfigurator().apply(primaryCANcoderConfiguration);

        final CANcoderConfiguration secondaryCANcoderConfiguration = new CANcoderConfiguration();
        secondaryCANcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        secondaryCANcoderConfiguration.MagnetSensor.MagnetOffset = constants.secondaryCANcoderOffsetRots();
        secondaryCANcoder.getConfigurator().apply(secondaryCANcoderConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                primaryCANcoderPosition,
                secondaryCANcoderPosition
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                motorDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                motor,
                primaryCANcoder,
                secondaryCANcoder
        );
    }

    @Override
    public void seedTurretPosition(final Rotation2d turretPosition) {
        final double turretPositionRots = turretPosition.getRotations();
        final double primaryGearing = constants.primaryCANcoderGearing();
        final double primaryAbsolutePosition = primaryCANcoder.getAbsolutePosition().getValueAsDouble() * primaryGearing;

        if (!MathUtil.isNear(primaryAbsolutePosition, turretPositionRots, 1e-6, 0, 1)) {
            DriverStation.reportError(String.format(
                    "Failed to seed turret position! Expected integer increment in position from: %.3f to %.3f",
                    Math.min(primaryAbsolutePosition, turretPositionRots),
                    Math.max(primaryAbsolutePosition, turretPositionRots)
            ), true);
            return;
        }

        // TODO: needs to try until OK, maybe needs more timeout
        Phoenix6Utils.reportIfNotOk(motor, primaryCANcoder.setPosition(turretPositionRots / primaryGearing));
    }

    @Override
    public void trackTurretPosition(final double turretPositionRots, final double turretVelocityRotsPerSec) {
        motor.setControl(positionVoltage
                .withSlot(0)
                .withPosition(turretPositionRots)
                .withVelocity(turretVelocityRotsPerSec)
        );
    }

    @Override
    public void toTurretPosition(final double turretPositionRots) {
        motor.setControl(motionMagicExpoVoltage
                .withSlot(1)
                .withPosition(turretPositionRots));
    }

    @Override
    public void toTurretVoltage(final double turretVolts) {
        motor.setControl(voltageOut.withOutput(turretVolts));
    }
}
