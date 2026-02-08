package frc.robot.subsystems.superstructure.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.RefreshAll;

public class TurretIOReal implements TurretIO {
    
    private final HardwareConstants.TurretConstants constants;
    
    private final TalonFX turretMotor;
    private final CANcoder leftEncoder;
    private final CANcoder rightEncoder;

    private final StatusSignal<Angle> turretPosition;
    private final StatusSignal<AngularVelocity> turretVelocity;
    private final StatusSignal<Voltage> turretVoltage;
    private final StatusSignal<Current> turretTorqueCurrent;
    private final StatusSignal<Temperature> turretDeviceTemp;

    private final StatusSignal<Angle> leftEncoderPosition;
    private final StatusSignal<Angle> rightEncoderPosition;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final PositionVoltage positionVoltage;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;

    
    public TurretIOReal( HardwareConstants.TurretConstants constants) {
        this.constants = constants;

        this.turretMotor = new TalonFX(constants.turretMotorID(), constants.CANBus().toPhoenix6CANBus());
        this.leftEncoder = new CANcoder(constants.leftEncoderID(), constants.CANBus().toPhoenix6CANBus());
        this.rightEncoder = new CANcoder(constants.rightEncoderID(), constants.CANBus().toPhoenix6CANBus());

        this.turretPosition = turretMotor.getPosition(false);
        this.turretVelocity = turretMotor.getVelocity(false);
        this.turretVoltage = turretMotor.getMotorVoltage(false);
        this.turretTorqueCurrent = turretMotor.getTorqueCurrent(false);
        this.turretDeviceTemp = turretMotor.getDeviceTemp(false);

        this.leftEncoderPosition = leftEncoder.getPosition(false);
        this.rightEncoderPosition = rightEncoder.getPosition(false);

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.positionVoltage = new PositionVoltage(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        RefreshAll.add(
                constants.CANBus(),
                turretPosition,
                turretVelocity,
                turretVoltage,
                turretTorqueCurrent,
                turretDeviceTemp
        );
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0 = new Slot0Configs()
                .withKS(0.016887)
                .withKV(10.263)
                .withKA(2.2613)
                .withKP(76.008)
                .withKD(40);
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        motorConfig.MotionMagic.MotionMagicExpo_kV = 9.263;
        motorConfig.MotionMagic.MotionMagicExpo_kA = 2.1;
        motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        motorConfig.CurrentLimits.StatorCurrentLimit = 50;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = constants.turretToMechanismGearing();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turretMotor.getConfigurator().apply(motorConfig);

        final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.MagnetOffset = constants.leftEncoderOffset();
        leftEncoder.getConfigurator().apply(encoderConfig);

        encoderConfig.MagnetSensor.MagnetOffset = constants.rightEncoderOffset();
        rightEncoder.getConfigurator().apply(encoderConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                turretPosition,
                turretVelocity,
                turretVoltage,
                turretTorqueCurrent,
                leftEncoderPosition,
                rightEncoderPosition
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                turretDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                4,
                turretMotor,
                leftEncoder,
                rightEncoder
        );
    }

    @Override
    public void updateInputs(final TurretIOInputs inputs) {
        inputs.turretPositionRots = turretPosition.getValueAsDouble();
        inputs.turretVelocityRotsPerSec = turretVelocity.getValueAsDouble();
        inputs.turretVoltage = turretVoltage.getValueAsDouble();
        inputs.turretTorqueCurrentAmps = turretTorqueCurrent.getValueAsDouble();
        inputs.turretTempCelsius = turretDeviceTemp.getValueAsDouble();

        inputs.leftPositionRots = leftEncoderPosition.getValueAsDouble();
        inputs.rightPositionRots = rightEncoderPosition.getValueAsDouble();
    }

    @Override
    public void toTurretPosition(final double positionRots) {
        turretMotor.setControl(motionMagicExpoVoltage.withPosition(positionRots));
    }

    @Override
    public void toTurretContinuousPosition(double positionRots) {
        turretMotor.setControl(positionVoltage.withPosition(positionRots));
    }

    @Override
    public void toTurretVoltage(double volts) {
        turretMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void toTurretTorqueCurrent(double torqueCurrent) {
        turretMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrent));
    }
}
