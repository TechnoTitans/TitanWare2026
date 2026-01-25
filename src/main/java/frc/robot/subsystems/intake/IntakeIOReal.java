package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.RefreshAll;

public class IntakeIOReal implements IntakeIO {
    private final HardwareConstants.IntakeConstants constants;

    private final TalonFX rollerMotor;
    private final TalonFX masterSliderMotor;
    private final TalonFX followerSliderMotor;
    private final CANcoder encoder;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerDeviceTemp;

    private final StatusSignal<Angle> masterSliderPosition;
    private final StatusSignal<AngularVelocity> masterSliderVelocity;
    private final StatusSignal<Voltage> masterSliderVoltage;
    private final StatusSignal<Current> masterSliderTorqueCurrent;
    private final StatusSignal<Temperature> masterSliderDeviceTemp;
    private final StatusSignal<Angle> encoderPosition;
    private final StatusSignal<AngularVelocity> encoderVelocity;

    private final StatusSignal<Angle> followerSliderPosition;
    private final StatusSignal<AngularVelocity> followerSliderVelocity;
    private final StatusSignal<Voltage> followerSliderVoltage;
    private final StatusSignal<Current> followerSliderTorqueCurrent;
    private final StatusSignal<Temperature> followerSliderDeviceTemp;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;
    private final Follower follower;


    public IntakeIOReal(final HardwareConstants.IntakeConstants constants) {
        this.constants = constants;

        this.rollerMotor = new TalonFX(constants.rollerMotorID(), constants.CANBus().toPhoenix6CANBus());
        this.masterSliderMotor = new TalonFX(constants.masterSliderMotorID(), constants.CANBus().toPhoenix6CANBus());
        this.followerSliderMotor = new TalonFX(constants.followerSliderMotorID(), constants.CANBus().toPhoenix6CANBus());
        this.encoder = new CANcoder(constants.encoderID(), constants.CANBus().toPhoenix6CANBus());

        this.rollerPosition = rollerMotor.getPosition(false);
        this.rollerVelocity = rollerMotor.getVelocity(false);
        this.rollerVoltage = rollerMotor.getMotorVoltage(false);
        this.rollerTorqueCurrent = rollerMotor.getTorqueCurrent(false);
        this.rollerDeviceTemp = rollerMotor.getDeviceTemp(false);

        this.masterSliderPosition = masterSliderMotor.getPosition(false);
        this.masterSliderVelocity = masterSliderMotor.getVelocity(false);
        this.masterSliderVoltage = masterSliderMotor.getMotorVoltage(false);
        this.masterSliderTorqueCurrent = masterSliderMotor.getTorqueCurrent(false);
        this.masterSliderDeviceTemp = masterSliderMotor.getDeviceTemp(false);
        this.encoderPosition = encoder.getPosition(false);
        this.encoderVelocity = encoder.getVelocity(false);

        this.followerSliderPosition = followerSliderMotor.getPosition(false);
        this.followerSliderVelocity = followerSliderMotor.getVelocity(false);
        this.followerSliderVoltage = followerSliderMotor.getMotorVoltage(false);
        this.followerSliderTorqueCurrent = followerSliderMotor.getTorqueCurrent(false);
        this.followerSliderDeviceTemp = followerSliderMotor.getDeviceTemp(false);

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);
        this.follower = new Follower(masterSliderMotor.getDeviceID(), MotorAlignmentValue.Opposed);

        RefreshAll.add(
                constants.CANBus(),
                rollerPosition,
                rollerVelocity,
                rollerVoltage,
                rollerTorqueCurrent,
                rollerDeviceTemp,
                masterSliderPosition,
                masterSliderVelocity,
                masterSliderVoltage,
                masterSliderTorqueCurrent,
                masterSliderDeviceTemp,
                followerSliderPosition,
                followerSliderVelocity,
                followerSliderVoltage,
                followerSliderTorqueCurrent,
                followerSliderDeviceTemp
        );
    }

    @Override
    public void config() {
        final TalonFXConfiguration rollerConfiguration = new TalonFXConfiguration();
        rollerConfiguration.Slot0 = new Slot0Configs()
                .withKS(0.01)
                .withKP(2);
        rollerConfiguration.CurrentLimits.StatorCurrentLimit = 40;
        rollerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        rollerConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        rollerConfiguration.CurrentLimits.SupplyCurrentLowerTime = 1;
        rollerConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rollerConfiguration.Feedback.SensorToMechanismRatio = constants.rollerGearing();
        rollerMotor.getConfigurator().apply(rollerConfiguration);

        final TalonFXConfiguration sliderMotorConfig = new TalonFXConfiguration();
        sliderMotorConfig.Slot0 = new Slot0Configs()
                .withKS(0.1)
                .withKG(0.01)
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKP(10)
                .withKD(0.1);
        sliderMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        sliderMotorConfig.MotionMagic.MotionMagicExpo_kV = 9.263;
        sliderMotorConfig.MotionMagic.MotionMagicExpo_kA = 2.1;
        sliderMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        sliderMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        sliderMotorConfig.CurrentLimits.StatorCurrentLimit = 50;
        sliderMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        sliderMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        sliderMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30;
        sliderMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        sliderMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        sliderMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        sliderMotorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        sliderMotorConfig.Feedback.SensorToMechanismRatio = 1;
        sliderMotorConfig.Feedback.RotorToSensorRatio = 112.84;
        sliderMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        sliderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        sliderMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        sliderMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        sliderMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        sliderMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        masterSliderMotor.getConfigurator().apply(sliderMotorConfig);
        sliderMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        followerSliderMotor.getConfigurator().apply(sliderMotorConfig);

        final CANcoderConfiguration sliderCANCoderConfig = new CANcoderConfiguration();
        sliderCANCoderConfig.MagnetSensor.MagnetOffset = constants.encoderOffset();
        sliderCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoder.getConfigurator().apply(sliderCANCoderConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                rollerPosition,
                rollerVelocity,
                rollerVoltage,
                rollerTorqueCurrent,
                masterSliderPosition,
                masterSliderVelocity,
                masterSliderVoltage,
                masterSliderTorqueCurrent,
                followerSliderPosition,
                followerSliderVelocity,
                followerSliderVoltage,
                followerSliderTorqueCurrent,
                encoderPosition,
                encoderVelocity
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                rollerDeviceTemp,
                masterSliderDeviceTemp,
                followerSliderDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                rollerMotor,
                masterSliderMotor,
                followerSliderMotor,
                encoder
        );
    }

    @Override
    public void updateInputs(final IntakeIOInputs inputs) {
        inputs.rollerPositionRots = rollerPosition.getValueAsDouble();
        inputs.rollerVelocityRotsPerSec = rollerVelocity.getValueAsDouble();
        inputs.rollerVoltage = rollerVoltage.getValueAsDouble();
        inputs.rollerTorqueCurrentAmps = rollerTorqueCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = rollerDeviceTemp.getValueAsDouble();

        inputs.masterSliderPositionRots = masterSliderPosition.getValueAsDouble();
        inputs.masterSliderVelocityRotsPerSec = masterSliderVelocity.getValueAsDouble();
        inputs.masterSliderVoltage = masterSliderVoltage.getValueAsDouble();
        inputs.masterSliderTorqueCurrentAmps = masterSliderTorqueCurrent.getValueAsDouble();
        inputs.masterSliderTempCelsius = masterSliderDeviceTemp.getValueAsDouble();

        inputs.followerSliderPositionRots = followerSliderPosition.getValueAsDouble();
        inputs.followerSliderVelocityRotsPerSec = followerSliderVelocity.getValueAsDouble();
        inputs.followerSliderVoltage = followerSliderVoltage.getValueAsDouble();
        inputs.followerSliderTorqueCurrentAmps = followerSliderTorqueCurrent.getValueAsDouble();
        inputs.followerSliderTempCelsius = followerSliderDeviceTemp.getValueAsDouble();

        inputs.encoderPositionRots = encoderPosition.getValueAsDouble();
        inputs.encoderVelocityRotsPerSec = encoderVelocity.getValueAsDouble();
    }

    @Override
    public void toRollerVelocity(final double velocityRotsPerSec) {
        rollerMotor.setControl(velocityTorqueCurrentFOC.withVelocity(velocityRotsPerSec));
    }

    @Override
    public void toRollerVoltage(final double volts) {
        rollerMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void toRollerTorqueCurrent(final double torqueCurrentAmps) {
        rollerMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
    }

    @Override
    public void toSliderPosition(double positionRots) {
        masterSliderMotor.setControl(motionMagicExpoVoltage.withPosition(positionRots));
        followerSliderMotor.setControl(follower);
    }

    @Override
    public void toSliderVoltage(double volts) {
        masterSliderMotor.setControl(voltageOut.withOutput(volts));
        followerSliderMotor.setControl(follower);
    }

    @Override
    public void toSliderTorqueCurrent(double torqueCurrent) {
        masterSliderMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrent));
        followerSliderMotor.setControl(follower);
    }
}