package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
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
    private final TalonFX sliderMotor;
    private final CANcoder sliderEncoder;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerDeviceTemp;

    private final StatusSignal<Angle> sliderPosition;
    private final StatusSignal<AngularVelocity> sliderVelocity;
    private final StatusSignal<Voltage> sliderVoltage;
    private final StatusSignal<Current> sliderTorqueCurrent;
    private final StatusSignal<Temperature> sliderDeviceTemp;
    private final StatusSignal<Angle> encoderPosition;
    private final StatusSignal<AngularVelocity> encoderVelocity;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;


    public IntakeIOReal(final HardwareConstants.IntakeConstants constants) {
        this.constants = constants;

        this.rollerMotor = new TalonFX(constants.rollerMotorID(), constants.CANBus().toPhoenix6CANBus());
        this.sliderMotor = new TalonFX(constants.sliderMotorID(), constants.CANBus().toPhoenix6CANBus());
        this.sliderEncoder = new CANcoder(constants.encoderID(), constants.CANBus().toPhoenix6CANBus());

        this.rollerPosition = rollerMotor.getPosition(false);
        this.rollerVelocity = rollerMotor.getVelocity(false);
        this.rollerVoltage = rollerMotor.getMotorVoltage(false);
        this.rollerTorqueCurrent = rollerMotor.getTorqueCurrent(false);
        this.rollerDeviceTemp = rollerMotor.getDeviceTemp(false);

        this.sliderPosition = sliderMotor.getPosition(false);
        this.sliderVelocity = sliderMotor.getVelocity(false);
        this.sliderVoltage = sliderMotor.getMotorVoltage(false);
        this.sliderTorqueCurrent = sliderMotor.getTorqueCurrent(false);
        this.sliderDeviceTemp = sliderMotor.getDeviceTemp(false);
        this.encoderPosition = sliderEncoder.getPosition(false);
        this.encoderVelocity = sliderEncoder.getVelocity(false);

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        RefreshAll.add(
                constants.CANBus(),
                rollerPosition,
                rollerVelocity,
                rollerVoltage,
                rollerTorqueCurrent,
                rollerDeviceTemp
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
        sliderMotorConfig.Feedback.FeedbackRemoteSensorID = sliderEncoder.getDeviceID();
        sliderMotorConfig.Feedback.SensorToMechanismRatio = 1;
        sliderMotorConfig.Feedback.RotorToSensorRatio = 112.84;
        sliderMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        sliderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        sliderMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        sliderMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        sliderMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        sliderMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        sliderMotor.getConfigurator().apply(sliderMotorConfig);

        final CANcoderConfiguration sliderCANCoderConfig = new CANcoderConfiguration();
        sliderCANCoderConfig.MagnetSensor.MagnetOffset = constants.encoderOffset();
        sliderCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        sliderEncoder.getConfigurator().apply(sliderCANCoderConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                rollerPosition,
                rollerVelocity,
                rollerVoltage,
                rollerTorqueCurrent,
                sliderPosition,
                sliderVelocity,
                sliderVoltage,
                sliderTorqueCurrent,
                encoderPosition,
                encoderVelocity
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                rollerDeviceTemp,
                sliderDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                rollerMotor,
                sliderMotor,
                sliderEncoder
        );
    }

    @Override
    public void updateInputs(final IntakeIOInputs inputs) {
        inputs.rollerPositionRots = rollerPosition.getValueAsDouble();
        inputs.rollerVelocityRotsPerSec = rollerVelocity.getValueAsDouble();
        inputs.rollerVoltage = rollerVoltage.getValueAsDouble();
        inputs.rollerTorqueCurrentAmps = rollerTorqueCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = rollerDeviceTemp.getValueAsDouble();

        inputs.sliderPositionRots = sliderPosition.getValueAsDouble();
        inputs.sliderVelocityRotsPerSec = sliderVelocity.getValueAsDouble();
        inputs.sliderVoltage = sliderVoltage.getValueAsDouble();
        inputs.sliderTorqueCurrentAmps = sliderTorqueCurrent.getValueAsDouble();
        inputs.sliderTempCelsius = sliderDeviceTemp.getValueAsDouble();
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
        sliderMotor.setControl(motionMagicExpoVoltage.withPosition(positionRots));
    }

    @Override
    public void toSliderVoltage(double volts) {
        sliderMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void toSliderTorqueCurrent(double torqueCurrent) {
        sliderMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrent));
    }
}