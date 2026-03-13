package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;

public class TurretIOReal implements TurretIO {
    private final HardwareConstants.TurretConstants constants;

    private final TalonFX turretMotor;
    private final CANcoder primaryEncoder;
    private final CANcoder secondaryEncoder;

    private final StatusSignal<Angle> turretPosition;
    private final StatusSignal<AngularVelocity> turretVelocity;
    private final StatusSignal<Voltage> turretVoltage;
    private final StatusSignal<Current> turretTorqueCurrent;
    private final StatusSignal<Temperature> turretDeviceTemp;

    private final StatusSignal<Angle> primaryEncoderPosition;
    private final StatusSignal<Angle> secondaryEncoderPosition;

    private final PositionVoltage positionVoltage;

    public TurretIOReal(HardwareConstants.TurretConstants constants) {
        this.constants = constants;

        this.turretMotor = new TalonFX(constants.turretMotorID(), constants.CANBus().toPhoenix6CANBus());
        this.primaryEncoder = new CANcoder(constants.primaryEncoderID(), constants.CANBus().toPhoenix6CANBus());
        this.secondaryEncoder = new CANcoder(constants.secondaryEncoderID(), constants.CANBus().toPhoenix6CANBus());

        this.turretPosition = turretMotor.getPosition(false);
        this.turretVelocity = turretMotor.getVelocity(false);
        this.turretVoltage = turretMotor.getMotorVoltage(false);
        this.turretTorqueCurrent = turretMotor.getTorqueCurrent(false);
        this.turretDeviceTemp = turretMotor.getDeviceTemp(false);

        this.secondaryEncoderPosition = secondaryEncoder.getPosition(true);
        this.primaryEncoderPosition = primaryEncoder.getPosition(true);

        this.positionVoltage = new PositionVoltage(0);

        RefreshAll.add(
                constants.CANBus(),
                turretPosition,
                turretVelocity,
                turretVoltage,
                turretTorqueCurrent,
                turretDeviceTemp,
                secondaryEncoderPosition,
                primaryEncoderPosition
        );
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0 = new Slot0Configs()
                .withKS(0.366)
                .withKV(5)
                .withKP(30)
                .withKD(0);
        motorConfig.CurrentLimits.StatorCurrentLimit = 70;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.motorToTurretGearing();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.forwardLimitRots();
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.reverseLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turretMotor.getConfigurator().apply(motorConfig);

        final CANcoderConfiguration primaryEncoderConfig = new CANcoderConfiguration();
        primaryEncoderConfig.MagnetSensor.MagnetOffset = constants.primaryEncoderOffset();
        primaryEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        primaryEncoder.getConfigurator().apply(primaryEncoderConfig);

        final CANcoderConfiguration secondaryEncoderConfig = new CANcoderConfiguration();
        secondaryEncoderConfig.MagnetSensor.MagnetOffset = constants.secondaryEncoderOffset();
        secondaryEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        secondaryEncoder.getConfigurator().apply(secondaryEncoderConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                turretPosition,
                turretVelocity,
                turretVoltage,
                turretTorqueCurrent,
                secondaryEncoderPosition,
                primaryEncoderPosition
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                turretDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                4,
                turretMotor,
                secondaryEncoder,
                primaryEncoder
        );
    }

    @Override
    public void updateInputs(final TurretIOInputs inputs) {
        inputs.turretPositionRots = turretPosition.getValueAsDouble();
        inputs.turretVelocityRotsPerSec = turretVelocity.getValueAsDouble();
        inputs.turretVoltage = turretVoltage.getValueAsDouble();
        inputs.turretTorqueCurrentAmps = turretTorqueCurrent.getValueAsDouble();
        inputs.turretTempCelsius = turretDeviceTemp.getValueAsDouble();

        inputs.primaryEncoderPositionRots = primaryEncoderPosition.getValueAsDouble();
        inputs.secondaryEncoderPositionRots = secondaryEncoderPosition.getValueAsDouble();
    }

    @Override
    public void toTurretContinuousPosition(final double positionRots, final double velocityRotsPerSec) {
        turretMotor.setControl(
                positionVoltage.withPosition(positionRots)
                        .withVelocity(velocityRotsPerSec)
                        .withSlot(0));
    }

    @Override
    public void setPosition(final double turretPositionRots) {
        Phoenix6Utils.reportIfNotOk(turretMotor, turretMotor.setPosition(turretPositionRots));
    }
}
