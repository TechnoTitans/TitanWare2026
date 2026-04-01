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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;

public class TurretIOReal implements TurretIO {
    private final HardwareConstants.TurretConstants constants;

    private final TalonFX motor;
    private final CANcoder primaryEncoder;
    private final CANcoder secondaryEncoder;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorDeviceTemp;

    private final StatusSignal<Angle> primaryEncoderPosition;
    private final StatusSignal<Angle> primaryEncoderAbsolutePosition;

    private final StatusSignal<Angle> secondaryEncoderPosition;
    private final StatusSignal<Angle> secondaryEncoderAbsolutePosition;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final PositionVoltage positionVoltage;
    private final VoltageOut voltageOut;

    public TurretIOReal(final HardwareConstants.TurretConstants constants) {
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        final CANBus p6Bus = bus.toPhoenix6CANBus();
        this.motor = new TalonFX(constants.motorID(), p6Bus);
        this.primaryEncoder = new CANcoder(constants.primaryCANcoderID(), p6Bus);
        this.secondaryEncoder = new CANcoder(constants.secondaryCANcoderID(), p6Bus);

        this.motorPosition = motor.getPosition(false);
        this.motorVelocity = motor.getVelocity(false);
        this.motorVoltage = motor.getMotorVoltage(false);
        this.motorTorqueCurrent = motor.getTorqueCurrent(false);
        this.motorDeviceTemp = motor.getDeviceTemp(false);

        this.primaryEncoderPosition = primaryEncoder.getPosition(false);
        this.primaryEncoderAbsolutePosition = primaryEncoder.getAbsolutePosition(false);

        this.secondaryEncoderPosition = secondaryEncoder.getPosition(false);
        this.secondaryEncoderAbsolutePosition = secondaryEncoder.getAbsolutePosition(false);

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.positionVoltage = new PositionVoltage(0);
        this.voltageOut = new VoltageOut(0);

        RefreshAll.add(
                bus,
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                motorDeviceTemp,
                primaryEncoderPosition,
                primaryEncoderAbsolutePosition,
                secondaryEncoderPosition,
                secondaryEncoderAbsolutePosition
        );

        config();
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0 = new Slot0Configs()
                .withKS(0.24265)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKV(3.067)
                .withKA(0.14961)
                .withKP(50)
                .withKD(4);
        motorConfig.Slot1 = new Slot1Configs()
                .withKS(0.24265)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKV(3.067)
                .withKA(0.14961)
                .withKP(50)
                .withKD(4);
        motorConfig.CurrentLimits.StatorCurrentLimit = 60;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 55;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 2.0;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.motorToGearboxGearing();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.forwardLimitRots();
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.reverseLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        Phoenix6Utils.tryUntilOk(motor, () -> motor.getConfigurator().apply(motorConfig));

        final CANcoderConfiguration primaryEncoderConfig = new CANcoderConfiguration();
        primaryEncoderConfig.MagnetSensor.MagnetOffset = constants.primaryCANcoderOffsetRots();
        primaryEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        primaryEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        Phoenix6Utils.tryUntilOk(
                primaryEncoder,
                () -> primaryEncoder.getConfigurator().apply(primaryEncoderConfig)
        );

        final CANcoderConfiguration secondaryEncoderConfig = new CANcoderConfiguration();
        secondaryEncoderConfig.MagnetSensor.MagnetOffset = constants.secondaryCANcoderOffsetRots();
        secondaryEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        secondaryEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        Phoenix6Utils.tryUntilOk(
                secondaryEncoder,
                () -> secondaryEncoder.getConfigurator().apply(secondaryEncoderConfig)
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                motorPosition,
                motorVelocity,
                motorVoltage,
                motorTorqueCurrent,
                primaryEncoderPosition,
                primaryEncoderAbsolutePosition,
                secondaryEncoderPosition,
                secondaryEncoderAbsolutePosition
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                motorDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                motor,
                primaryEncoder,
                secondaryEncoder
        );
    }

    @Override
    public void updateInputs(final TurretIOInputs inputs) {
        inputs.turretPositionRots = motorPosition.getValueAsDouble();
        inputs.turretVelocityRotsPerSec = motorVelocity.getValueAsDouble();
        inputs.turretVoltage = motorVoltage.getValueAsDouble();
        inputs.turretTorqueCurrentAmps = motorTorqueCurrent.getValueAsDouble();
        inputs.turretTempCelsius = motorDeviceTemp.getValueAsDouble();

        inputs.primaryEncoderPositionRots = primaryEncoderPosition.getValueAsDouble();
        inputs.primaryEncoderAbsolutePositionRots = primaryEncoderAbsolutePosition.getValueAsDouble();

        inputs.secondaryEncoderPositionRots = secondaryEncoderPosition.getValueAsDouble();
        inputs.secondaryEncoderAbsolutePositionRots = secondaryEncoderAbsolutePosition.getValueAsDouble();
    }

    @Override
    public void toTurretPosition(final double positionRots) {
        motor.setControl(
                motionMagicExpoVoltage
                    .withPosition(positionRots)
                    .withSlot(0)
        );
    }

    @Override
    public void toTurretContinuousPosition(final double positionRots, final double velocityRotsPerSec) {
        motor.setControl(
                positionVoltage
                        .withPosition(positionRots)
                        .withVelocity(velocityRotsPerSec)
                        .withSlot(1)
        );
    }

    @Override
    public void toTurretVoltage(final double volts) {
        motor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void seedTurretPosition(final Rotation2d turretPosition) {
        final double turretPositionRots = turretPosition.getRotations();
        Phoenix6Utils.tryUntilOk(motor, 10, () -> motor.setPosition(turretPositionRots));
    }
}
